#include <vector>
#include <algorithm>

// 中值滤波器
class MedianFilter
{
public:
    MedianFilter(int window_size) : window_size(window_size) {}

    double filter(double new_value)
    {
        if (values.size() >= window_size)
        {
            values.pop_front();
        }
        values.push_back(new_value);

        values.sort();
        auto it = values.begin();
        std::advance(it, values.size() / 2);
        return *it;
    }

private:
    int window_size;
    std::list<double> values;
};

class MovingAverageFilter
{
public:
    MovingAverageFilter(int window_size) : window_size(window_size) {}

    double filter(double new_value)
    {
        if (values.size() >= window_size)
        {
            values.pop_front();
        }
        values.push_back(new_value);

        double sum = 0.0;
        for (double value : values)
        {
            sum += value;
        }
        return sum / values.size();
    }

private:
    int window_size;
    std::list<double> values;
};