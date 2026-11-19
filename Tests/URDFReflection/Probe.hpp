#pragma once
// Deliberately unrelated to FineMote's device names: tests automatic discovery.
template<int N> struct ReflectionConfig { float original; };
template<> struct ReflectionConfig<7> { const float gain = 2; int count = 3; };
template<int N> struct ReflectionProbe {
    ReflectionProbe(ReflectionConfig<N> config, int tail = 9) : gain(config.gain), count(config.count), tail(tail) {}
    const float gain;
    const int count;
    const int tail;
};
