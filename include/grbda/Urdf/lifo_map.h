#ifndef URDF_INTERFACE_LIFO_MAP_H
#define URDF_INTERFACE_LIFO_MAP_H

#include <vector>
#include <unordered_map>

namespace urdf
{
    // A custom variant of std::map that differs in that the values are stored in a vector 
    // in the order they were inserted
    template <typename Key, typename Value>
    class LifoMap
    {
    public:
        void insert(const Key &key, const Value &value)
        {
            data.push_back(value);
            map[key] = data.size() - 1;
        }
        void insert(const std::pair<Key, Value> &pair)
        {
            data.push_back(pair.second);
            map[pair.first] = data.size() - 1;
        }

        Value &operator[](const Key &key) { return data[map[key]]; }
        Value &operator[](const size_t &index) { return data[index]; }

        const Value &at(const Key &key) const { return data[map.at(key)]; }

        size_t keyIndex(const Key &key) const { return map.at(key); }

        void clear()
        {
            data.clear();
            map.clear();
        }

        bool empty() const { return data.empty(); }

        size_t size() const { return data.size(); }

        using iterator = typename std::vector<Value>::iterator;
        using const_iterator = typename std::vector<Value>::const_iterator;

        const_iterator begin() const { return const_iterator(data.begin()); }
        iterator begin() { return iterator(data.begin()); }

        const_iterator end() const { return const_iterator(data.end()); }
        iterator end() { return iterator(data.end()); }

        const_iterator find(const Key &key) const
        {
            if (map.find(key) != map.end())
            {
                return const_iterator(data.begin() + map.at(key));
            }
            else
            {
                return end();
            }
        }

        iterator find(const Key &key)
        {
            if (map.find(key) != map.end())
            {
                return iterator(data.begin() + map.at(key));
            }
            else
            {
                return end();
            }
        }

    private:
        std::vector<Value> data;
        std::unordered_map<Key, size_t> map;
    };
}

#endif // URDF_INTERFACE_LIFO_MAP_H
