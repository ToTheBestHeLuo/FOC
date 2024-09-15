#ifndef _MC_HVSENSORLESS_H_
#define _MC_HVSENSORLESS_H_

#include "mcVar.h"

extern void NonlinearFluxObsProcess(_FORCE MC_MotorPar* motor,_FORCE NonlinearFluxObsHandler* pNLFO,_FORCE PIC* sp,Components2* uAlphaBeta,Components2* iAlphaBeta);
extern void LuenbergerObs(_FORCE Components2* iAlphaBeta,_FORCE Components2* uAlphaBeta,_FORCE LuenbergerObsHandler* obs);

#endif




