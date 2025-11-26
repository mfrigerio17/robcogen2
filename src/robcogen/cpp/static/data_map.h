#ifndef RCG2_DATA_MAP_H_
#define RCG2_DATA_MAP_H_

namespace rcg2 {

template<typename T, unsigned int Size, typename ItemID> struct DataMap
{
    DataMap() {};
    DataMap(const T& defaultValue) {
        assigndata(defaultValue);
    }
    DataMap(const DataMap& rhs) {
        copydata(rhs);
    }
    DataMap& operator=(const DataMap& rhs) {
        if(&rhs != this) {
            copydata(rhs);
        }
        return *this;
    }
    DataMap& operator=(const T& rhs) {
        assigndata(rhs);
        return *this;
    }
    T& operator[](ItemID which) {
        return data[which];
    }
    const T& operator[](ItemID which) const {
        return data[which];
    }
private:
    T data[Size];

    void copydata(const DataMap& rhs) {
        std::memcpy(static_cast<void*>(data), rhs.data, sizeof(data));
    }
    void assigndata(const T& value) {
        std::fill(data, data+Size, value);
    }
};



}

#endif
