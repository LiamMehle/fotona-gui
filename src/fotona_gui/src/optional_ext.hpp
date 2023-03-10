#include <optional>

template<typename T, typename F>
constexpr inline
auto map(std::optional<T> const optional_value, F op) -> decltype(op(*optional_value)) {
    if (optional_value.has_value())
        op(*optional_value);
}