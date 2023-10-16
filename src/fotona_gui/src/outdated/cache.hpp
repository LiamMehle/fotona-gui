template<typename O, typename F, typename T>
struct Cached {
    O object;
    F action;
    T cached_value;
    void operator(T const updated_value) {
        if (updated_value == this->cached_value)
            return;
        this->cached_value = updated_value;
        action(object, updated_value);
    }
};
