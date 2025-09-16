#include "cuda_utils.h"

#include <sys/time.h>
#include <unistd.h>

#include <numeric>

size_t shapeToNumElement(Dims32 dim)
{
  return std::accumulate(dim.d, dim.d + dim.nbDims, 1, std::multiplies<size_t>());
}

size_t dataTypeToSize(DataType dataType)
{
  switch ((int)dataType)
  {
  case int(DataType::kFLOAT):
    return 4;
  case int(DataType::kHALF):
    return 2;
  case int(DataType::kINT8):
    return 1;
  case int(DataType::kINT32):
    return 4;
  case int(DataType::kBOOL):
    return 1;
  case int(DataType::kUINT8):
    return 1;
  default:
    return 4;
  }
}
std::string shapeToString(Dims32 dim)
{
  std::string output("(");
  if (dim.nbDims == 0)
  {
    return output + std::string(")");
  }
  for (int i = 0; i < dim.nbDims - 1; ++i)
  {
    output += std::to_string(dim.d[i]) + std::string(", ");
  }
  output += std::to_string(dim.d[dim.nbDims - 1]) + std::string(")");
  return output;
}

std::vector<size_t> shapeToVec(Dims32 dim)
{
  std::vector<size_t> output;
  for (int i = 0; i < dim.nbDims; ++i)
  {
    output.push_back(dim.d[i]);
  }
  return output;
}

// 数据类型转字符串
std::string dataTypeToString(DataType dataType)
{
  switch (dataType)
  {
  case DataType::kFLOAT:
    return std::string("FP32 ");
  case DataType::kHALF:
    return std::string("FP16 ");
  case DataType::kINT8:
    return std::string("INT8 ");
  case DataType::kINT32:
    return std::string("INT32");
  case DataType::kBOOL:
    return std::string("BOOL ");
  case DataType::kUINT8:
    return std::string("UINT8 ");
  default:
    return std::string("Unknown");
  }
}

std::string getFormatString(TensorFormat format)
{
  switch (format)
  {
  case TensorFormat::kLINEAR:
    return std::string("LINE ");
  case TensorFormat::kCHW2:
    return std::string("CHW2 ");
  case TensorFormat::kHWC8:
    return std::string("HWC8 ");
  case TensorFormat::kCHW4:
    return std::string("CHW4 ");
  case TensorFormat::kCHW16:
    return std::string("CHW16");
  case TensorFormat::kCHW32:
    return std::string("CHW32");
  case TensorFormat::kHWC:
    return std::string("HWC  ");
  case TensorFormat::kDLA_LINEAR:
    return std::string("DLINE");
  case TensorFormat::kDLA_HWC4:
    return std::string("DHWC4");
  case TensorFormat::kHWC16:
    return std::string("HWC16");
  default:
    return std::string("None ");
  }
}

ICudaEngine* InitEngine(const std::string& engine_file, Logger& gLogger) {
  initLibNvInferPlugins(nullptr, "");
  ICudaEngine* engine = nullptr;

  if (access(engine_file.c_str(), F_OK) == 0) {
    std::ifstream engineFile(engine_file, std::ios::binary);
    long int fsize = 0;

    engineFile.seekg(0, engineFile.end);
    fsize = engineFile.tellg();
    engineFile.seekg(0, engineFile.beg);
    std::vector<char> engineString(fsize);
    engineFile.read(engineString.data(), fsize);
    if (engineString.size() == 0) {
      std::cout << "Failed getting serialized engine!" << std::endl;
      return nullptr;
    }
    std::cout << "Succeeded getting serialized engine!" << std::endl;

    IRuntime* runtime{createInferRuntime(gLogger)};
    engine = runtime->deserializeCudaEngine(engineString.data(), fsize);
    if (engine == nullptr) {
    std::cout << "Failed loading engine!" << std::endl;
    return nullptr;
  }
  std::cout << "Succeeded loading engine!" << std::endl;
  return engine;
  } else {
    std::cout << "Failed finding " << engine_file << " file!" << std::endl;
    return nullptr;
  }
}
