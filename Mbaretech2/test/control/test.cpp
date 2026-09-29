#include "combatFsm.h"
#include <cassert>
#include <iostream>

SensorSnapshot snapshot(unsigned opening, uint32_t now = 0) {
    SensorSnapshot s;
    s.valid = true; s.sampledAtMs = now;
    s.dip[DIP_E] = opening & 8; s.dip[DIP_A] = opening & 4;
    s.dip[DIP_B] = opening & 2; s.dip[DIP_C] = opening & 1;
    return s;
}
MotorCommand tick(CombatFsm& f, SensorSnapshot& s, uint32_t now, bool start = true) {
    s.sampledAtMs = now;
    return f.step(s, start, now);
}
void stopped(MotorCommand c) { assert(c.left == 0 && c.right == 0); }
int main() {
    // Every DIP opening can be stopped before, during and after timed phases.
    const State openings[] = {FORWARD,FORWARD,BRAKE,BRAKE,TURN_LEFT_90,TURN_RIGHT_90,
        TURN_180,L_MOVEMENT_45,R_MOVEMENT_45,GIRO_U_L,GIRO_U_R,GIRO_U_L_LONG,
        GIRO_U_R_LONG,SHORT_LEFT_MOVE,SHORT_RIGHT_MOVE,GIRO_U_L};
    for (unsigned choice = 0; choice < 16; ++choice) {
        for (uint32_t stopAt : {1u, 61u, 212u, 1001u, 2001u}) {
            CombatFsm f; auto s = snapshot(choice);
            tick(f,s,0); assert(f.state() == openings[choice]);
            for (uint32_t t=1; t<stopAt; ++t) tick(f,s,t);
            stopped(tick(f,s,stopAt,false)); assert(f.state()==IDLE);
            stopped(tick(f,s,stopAt+1,false));
        }
    }
    { // No motion until valid acquisition; stale data stops an active turn.
        CombatFsm f; auto s = snapshot(4); s.valid=false;
        stopped(f.step(s,true,0)); assert(f.state()==IDLE);
        s.valid=true; auto c=f.step(s,true,0); assert(c.left<0 && c.right>0);
        stopped(f.step(s,true,SENSOR_MAX_AGE_MS+1)); assert(f.state()==IDLE);
    }
    { // A target cancels a turn without a later BRAKE overwriting FORWARD.
        CombatFsm f; auto s=snapshot(4); tick(f,s,0);
        s.ir[TOP_MID]=true; tick(f,s,1); assert(f.state()==FORWARD);
        auto c=tick(f,s,2); assert(c.left>0 && c.right>0);
    }
    { // Border wins even with a target; repeated samples do not reset retreat.
        CombatFsm f; auto s=snapshot(11); tick(f,s,0);
        s.line[0]=true; s.ir[TOP_MID]=true;
        auto c=tick(f,s,1); assert(f.state()==LINE_RETREAT && c.left<0 && c.right<0);
        c=tick(f,s,81); assert(f.state()==LINE_RETREAT && c.left<0);
        s.line[0]=false; tick(f,s,82); assert(f.state()==BRAKE);
    }
    { // Compound L: left turn, straight, right turn; deadlines are per phase.
        CombatFsm f; auto s=snapshot(7);
        auto c=tick(f,s,0); assert(c.left<0 && c.right>0);
        c=tick(f,s,TURN_LEFT_45_DELAY-1); assert(c.left<0);
        c=tick(f,s,TURN_LEFT_45_DELAY); assert(c.left>0 && c.right>0);
        c=tick(f,s,TURN_LEFT_45_DELAY+150); assert(c.left>0 && c.right<0);
        stopped(tick(f,s,TURN_LEFT_45_DELAY+150+TURN_RIGHT_90_DELAY));
        assert(f.state()==BRAKE);
    }
    { // Compound R mirrors L, and stop discards the remaining phases.
        CombatFsm f; auto s=snapshot(8);
        auto c=tick(f,s,0); assert(c.left>0 && c.right<0);
        c=tick(f,s,TURN_RIGHT_45_DELAY); assert(c.left>0 && c.right>0);
        stopped(tick(f,s,TURN_RIGHT_45_DELAY+1,false));
        c=tick(f,s,TURN_RIGHT_45_DELAY+2); assert(c.left>0 && c.right<0);
    }
    { // Turkish waiting and pulse do not block stop or border checks.
        CombatFsm f; auto s=snapshot(2); stopped(tick(f,s,0));
        stopped(tick(f,s,TURKISH_TIME-1));
        auto c=tick(f,s,TURKISH_TIME); assert(c.left>0 && c.right>0);
        stopped(tick(f,s,TURKISH_TIME+TURKISH_DELAY));
    }
    { // Snake uses timed alternating output and remains interruptible.
        CombatFsm f; auto s=snapshot(1); s.ir[TOP_MID]=true;
        auto a=tick(f,s,0); auto b=tick(f,s,SHORT_RIGHT_DELAY+10);
        assert(a.left>a.right && b.left<b.right);
        s.line[1]=true; auto c=tick(f,s,SHORT_RIGHT_DELAY+11); assert(c.left<0 && c.right<0);
    }
    { // Unsigned elapsed comparisons tolerate millis wraparound.
        CombatFsm f; auto s=snapshot(4,UINT32_MAX-20);
        auto c=tick(f,s,UINT32_MAX-20); assert(c.left<0);
        c=tick(f,s,10); assert(c.left<0);
        stopped(tick(f,s,TURN_LEFT_90_DELAY-21)); assert(f.state()==BRAKE);
    }
    std::cout << "PASS: openings, stop, stale data, target cancellation, border priority, compound phases, Turkish, snake, wraparound\n";
}
