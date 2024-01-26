#ifndef __FILTER_H_
#define __FILTER_H_


#define LowPass_Filter_Count 10
class LowPass_Filter
{
public:
    LowPass_Filter(void)
    {
        for(int i = 0; i < LowPass_Filter_Count; i++)
        {
            this->last_data[i] = 0.0f;
        }
    }

    void set(float target)
    {
        for(int i = 0; i < LowPass_Filter_Count; i++)
        {
            last_data[i] = target;
        }
    }

    //新插入数据
    void insert(float new_data)
    {
        for(int i = 0; i < LowPass_Filter_Count - 1; i++)
        {
            this->last_data[i] = this->last_data[i+1];
        }
        this->last_data[LowPass_Filter_Count-1] = new_data;
    }

    float getData(void)
    {
        float all = 0;
        for(int i = 0; i < LowPass_Filter_Count; i++)
        {
            all += last_data[i];
        }

        return (all / LowPass_Filter_Count); 
    }

    ~LowPass_Filter()
    {

    }

private:
    float last_data[LowPass_Filter_Count];
};

#endif
