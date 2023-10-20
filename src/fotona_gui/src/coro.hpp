#include <coroutine.h>

struct Task
{
    struct promise_type
    {
        using Handle = std::coroutine_handle<promise_type>;

        task get_return_object() { return Task{from_promise(*this)}; }
        std::suspend_never initial_suspend() { return {}; }
        std::suspend_never final_suspend() noexcept { return {}; }
        void return_void() {}
        void unhandled_exception() {}
    };
    ~Task() {
        if (coro_ && !coro.done())
            coro_.destroy();
    }
};
