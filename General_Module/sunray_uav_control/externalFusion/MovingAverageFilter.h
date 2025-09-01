
// 滑动平均滤波器
class MovingAverageFilter
{
public:
    MovingAverageFilter(int size = 5)
    {
        this->size = size;
        this->data = new double[size];
        this->sum = 0;
        this->count = 0;
        this->index = 0;
    }

    ~MovingAverageFilter()
    {
        delete[] data;
    }

    void setSize(int size)
    {
        this->size = size;
        delete[] data;
        this->data = new double[size];
        this->sum = 0;
        this->count = 0;
        this->index = 0;
    }

    void addData(double value)
    {

        if (count < size)
        {
            sum += value;
            data[count++] = value;
        }
        else
        {
            sum -= data[index];
            sum += value;
            data[index] = value;
            index = (index + 1) % size;
        }
    }

    double getAverage()
    {
        return sum / count;
    }

    double filter(double value)
    {
        addData(value);
        return getAverage();
    }

private:
    int size;
    double *data;
    double sum;
    int count;
    int index;
};

