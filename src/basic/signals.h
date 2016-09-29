#ifndef SIGNALS_H_INCLUDED
#define SIGNALS_H_INCLUDED

#include <csignal>
#include <cerrno>

class Signals {
public:
    Signals(void (*fp)(int))
    {
        new_action.sa_handler = fp;
        sigemptyset(&new_action.sa_mask);
        new_action.sa_flags = 0;

        if (sigaction(SIGINT, &new_action, &old_action_int) == -1) {
            printf("Error sigaction --> errno = %d - %s\n", errno, strerror(errno));
            exit(-1);
        }
        if (sigaction(SIGTERM, &new_action, &old_action_term) == -1) {
            printf("Error sigaction --> errno = %d - %s\n", errno, strerror(errno));
            exit(-1);
        }
        if (sigaction(SIGUSR1, &new_action, &old_action_usr1) == -1) {
            printf("Error sigaction --> errno = %d - %s\n", errno, strerror(errno));
            exit(-1);
        }
    }

    ~Signals()
    {
        if (sigaction(SIGINT, &old_action_int, NULL) == -1) {
            printf("Error sigaction --> errno = %d - %s\n", errno, strerror(errno));
            exit(-1);
        }
        if (sigaction(SIGTERM, &old_action_term, NULL) == -1) {
            printf("Error sigaction --> errno = %d - %s\n", errno, strerror(errno));
            exit(-1);
        }
        if (sigaction(SIGUSR1, &old_action_usr1, NULL) == -1) {
            printf("Error sigaction --> errno = %d - %s\n", errno, strerror(errno));
            exit(-1);
        }
    }
private:
    struct sigaction new_action;
    struct sigaction old_action_int;
    struct sigaction old_action_term;
    struct sigaction old_action_usr1;
};


#endif // SIGNALS_H_INCLUDED
