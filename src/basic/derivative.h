#ifndef DERIVATIVE_H_INCLUDED
#define DERIVATIVE_H_INCLUDED

#include <deque>

template <typename T>
class ChangeLimiter {
	T last_pos, last_vel;
	const T max_value;
public:
	ChangeLimiter(T max_value) : last_pos(), last_vel(), max_value(max_value) {}

	T step(T current) {
	    const double vel = current - last_pos;
		last_pos = last_pos + common::clip((vel + last_vel)/2, max_value);
	    last_vel = vel;
		return last_pos;
	}
};


struct LowpassDiff {

    double last = .0, velocity = .0;
    const double inv_dt;
    ChangeLimiter<double> limiter;

    LowpassDiff(double initial, double timestep, double scale, double limit)
    : inv_dt(scale/timestep)
    , limiter(limit)
    {
        assert(limit > 0.0);
        reset(initial);
    }

    void reset(double current) { last = current; }

    void derive(double current) {
        velocity = limiter.step( (current - last) * inv_dt );
        last = current;
    }

    double get(void) const { return velocity; }
};


struct DerivedWithDelay {

    std::deque<double> buffer;

    double last = .0;
    const double inv_dt;
    unsigned Ndelay;

    DerivedWithDelay(double initial, double timestep, double scale, unsigned Ndelay)
    : inv_dt(scale/timestep)
    , Ndelay(Ndelay)
    {
        reset(initial);
    }

    void reset(double current) {
        last = current;
        buffer.clear();
        for (unsigned i = 0; i < Ndelay; ++i)
            buffer.push_front(.0);
        assert(buffer.size() == Ndelay);
    }

    void derive(double current) {
        buffer.pop_back();
        const double velocity = (current - last) * inv_dt;
        buffer.push_front(velocity); // fill in new value
        last = current;
    }

    double get(void) const { return buffer.back(); }
};

struct Derived {

    double last = .0;
    double velocity = .0;
    const double inv_dt;

    Derived(double initial, double timestep, double scale)
    : inv_dt(scale/timestep)
    {
        reset(initial);
    }

    void reset(double current) { last = current; }

    void derive(double current) {
        velocity = (current - last) * inv_dt;
        last = current;
    }

    double get(void) const { return velocity; }
};


#endif // DERIVATIVE_H_INCLUDED
