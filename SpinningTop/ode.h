#pragma once
#pragma once
#include <vector>
#include <functional>

namespace ODE
{
    template<typename T, std::size_t N>
    std::array<T, N> rk4(std::array<T, N> u0, T t0, T dt, std::function<std::array<T, N>(T t, std::array<T, N> u)> derivative)
    {
        T h = dt / 2.0;
        T time = t0 + h;

        std::array<T, N> u1, u2, u3;


        std::array<T, N> k1 = derivative(t0, u0);

        for (size_t i = 0; i < N; i++)
            u1[i] = u0[i] + h*k1[i];

        std::array<T, N> k2 = derivative(time, u1);

        for (size_t i = 0; i < N; i++)
            u2[i] = u0[i] + h*k2[i];

        std::array<T, N> k3 = derivative(time, u2);

        for (size_t i = 0; i < N; i++)
            u3[i] = u0[i] + h*k3[i];

        std::array<T, N> k4 = derivative(time, u3);

        std::array<T, N> u;
        for (size_t i = 0; i < u0.size(); i++)
            u[i] = u0[i] + dt * (k1[i] + 2.0 * (k2[i] + k3[i]) + k4[i]) / 6.0;

        return u;
    }
}

