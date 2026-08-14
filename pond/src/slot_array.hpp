#include <cstdint>
#include <optional>
#include <utility>
#include <vector>
#include <iterator>

template <typename T>
class SlotArray
{
public:
    SlotArray() = default;

    explicit SlotArray(uint32_t initial_capacity) : elements(initial_capacity) {}

    SlotArray(const SlotArray&) = default;
    SlotArray(SlotArray&& other) noexcept
        : elements(std::move(other.elements)), free_index(other.free_index)
    {
        other.free_index = 0;
    }

    SlotArray& operator=(const SlotArray&) = default;

    SlotArray& operator=(SlotArray&& other) noexcept
    {
        if (this != &other)
        {
            elements = std::move(other.elements);
            free_index = other.free_index;
            other.free_index = 0;
        }

        return *this;
    }

    ~SlotArray() = default;

    T& operator[](uint32_t index) noexcept
    {
        return elements[index].value();
    }

    const T& operator[](uint32_t index) const noexcept
    {
        return elements[index].value();
    }

    T& at(uint32_t index)
    {
        return elements.at(index).value();
    }

    const T& at(uint32_t index) const
    {
        return elements.at(index).value();
    }

    template <typename... Args>
    uint32_t emplace(Args&&... args)
    {
        ensure_free_slot();

        const uint32_t index = free_index;

        elements[index].emplace(std::forward<Args>(args)...);

        advance_free_index();

        return index;
    }

    uint32_t insert(const T& value)
    {
        return emplace(value);
    }

    uint32_t insert(T&& value)
    {
        return emplace(std::move(value));
    }

    void release_slot(uint32_t slot)
    {
        elements.at(slot).reset();

        if (slot < free_index)
            free_index = slot;
    }

    [[nodiscard]]
    uint32_t get_length() const noexcept
    {
        return elements.size();
    }

    [[nodiscard]]
    bool is_used(uint32_t slot) const noexcept
    {
        if (slot >= elements.size()) return false;
        return elements[slot].has_value();
    }

private:
    void ensure_free_slot()
    {
        if (free_index != elements.size())
            return;

        elements.resize(elements.size() == 0 ? 1 : elements.size() * 2);
    }

    void advance_free_index() noexcept
    {
        while (free_index < elements.size() && elements[free_index].has_value())
            ++free_index;
    }

public:
    class iterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using pointer = T*;
        using reference = T&;

        iterator(typename std::vector<std::optional<T>>::iterator current,
                 typename std::vector<std::optional<T>>::iterator end)
            : current(current), end(end)
        {
            skip_empty();
        }

        reference operator*() const
        {
            return current->value();
        }

        pointer operator->() const
        {
            return &current->value();
        }

        iterator& operator++()
        {
            ++current;
            skip_empty();
            return *this;
        }

        iterator operator++(int)
        {
            iterator copy = *this;
            ++(*this);
            return copy;
        }

        bool operator==(const iterator& other) const
        {
            return current == other.current;
        }

        bool operator!=(const iterator& other) const
        {
            return !(*this == other);
        }

    private:
        void skip_empty()
        {
            while (current != end && !current->has_value())
                ++current;
        }

        typename std::vector<std::optional<T>>::iterator current;
        typename std::vector<std::optional<T>>::iterator end;
    };

    class const_iterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = const T;
        using difference_type = std::ptrdiff_t;
        using pointer = const T*;
        using reference = const T&;

        const_iterator(typename std::vector<std::optional<T>>::const_iterator current,
                       typename std::vector<std::optional<T>>::const_iterator end)
            : current(current), end(end)
        {
            skip_empty();
        }

        reference operator*() const
        {
            return current->value();
        }

        pointer operator->() const
        {
            return &current->value();
        }

        const_iterator& operator++()
        {
            ++current;
            skip_empty();
            return *this;
        }

        const_iterator operator++(int)
        {
            const_iterator copy = *this;
            ++(*this);
            return copy;
        }

        bool operator==(const const_iterator& other) const
        {
            return current == other.current;
        }

        bool operator!=(const const_iterator& other) const
        {
            return !(*this == other);
        }

    private:
        void skip_empty()
        {
            while (current != end && !current->has_value())
                ++current;
        }

        typename std::vector<std::optional<T>>::const_iterator current;
        typename std::vector<std::optional<T>>::const_iterator end;
    };

    iterator begin()
    {
        return iterator(elements.begin(), elements.end());
    }

    iterator end()
    {
        return iterator(elements.end(), elements.end());
    }

    const_iterator begin() const
    {
        return const_iterator(elements.begin(), elements.end());
    }

    const_iterator end() const
    {
        return const_iterator(elements.end(), elements.end());
    }

    const_iterator cbegin() const
    {
        return const_iterator(elements.cbegin(), elements.cend());
    }

    const_iterator cend() const
    {
        return const_iterator(elements.cend(), elements.cend());
    }

private:
    std::vector<std::optional<T>> elements;
    uint32_t free_index = 0;
};