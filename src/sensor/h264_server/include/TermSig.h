#ifndef TERMSIG_H
#define TERMSIG_H

#include <csignal>

namespace TermSignal{

static bool keepRunning = false;

void sig_handler(int sig){
    if ( sig == SIGINT)
    {
        keepRunning = false;
    }
};
void init(){
    keepRunning = true;
    signal( SIGINT, sig_handler );
}

bool ok(){
    return keepRunning;
}

}

#endif
