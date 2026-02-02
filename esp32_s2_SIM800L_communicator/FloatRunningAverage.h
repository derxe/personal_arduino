#pragma once


template <int BUF_LEN>
class FloatRunningAverage {
public:
    using ReadFunc = float (*)();   // function used to read the value that is going to be saved in the buffer

    FloatRunningAverage(ReadFunc fn)
        : readFn(fn), head(0), count(0), sum(0.0f) {}

    float log() {
        float val = readFn();   // call the function
        addSample(val);
        return val;
    }

    // return average value 
    float get() {
        if (count == 0) return log();
        return sum / (float)count;
    }
    
    int size() {
    	return count;
    }

    bool isEmpty() const { return count == 0; }

    void reset() {
        head = 0;
        count = 0;
        sum = 0.0f;
    }

private:
    float buffer[BUF_LEN];
    int head;
    int count;
    float sum;

    ReadFunc readFn;

    void addSample(float val) {
        if (count < BUF_LEN) {
            buffer[head] = val;
            sum += val;
            head = (head + 1) % BUF_LEN;
            count++;
        } else {
            float old = buffer[head];
            sum -= old;
            buffer[head] = val;
            sum += val;
            head = (head + 1) % BUF_LEN;
        }
    }
};























