/* Copyright (C) 2020 Joshua J. Daymude, Robert Gmyr, and Kristian Hinnenthal.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

#include "alg/test.h"
#include <iostream>
#include <Windows.h>
using namespace std;

TestParticle::TestParticle(const Node head, const int globalTailDir,
                                   const int orientation, AmoebotSystem &system,
                                   State state)
  : AmoebotParticle(head, globalTailDir, orientation, system),
    state(state),
    moveDir(-1),
    moveExpandDir(-1){}

void TestParticle::activate() {
    if (isContracted()) {
        if (state == State::Leader) {
            // Choose a random move direction not occupied by the follower.
            int dir = randDir();
            moveDir = randDir();
            while (hasNbrAtLabel(moveDir)) {
              moveDir = randDir();
            }
            expand(moveDir);
        }
    } else {  // isExpanded().
        if (state == State::Leader) {
            // Pull handover with the follower if it is contracted.
            int nbrLabel = labelOfFirstNbrInState({State::Follower});
            TestParticle& nbr = nbrAtLabel(nbrLabel);
            int taildir = (tailDir() + 3) % 6;
            if (nbr.isContracted()) {

                if(isHeadLabel(nbrLabel)){
                    nbrLabel=(nbrLabel+1)%10;
                }
                pull(nbrLabel);

                nbr.moveDir = dirToNbrDir(nbr, moveDir);
                nbr.moveExpandDir = calculateMoveExpDir(taildir, (nbr.tailDir() + 3) % 6);
            }
        }else if (state == State::Follower) {
          contractTail();
          moveDir = labelOfFirstNbrInState({State::Leader});
        }
    }
}

int TestParticle::calculateMoveExpDir(int tailDir, int nbrTailDir){
    int nbrMoveExpDir = tailDir;
    switch(tailDir) {
        case 0:
            if(nbrTailDir == 2){
                nbrMoveExpDir += 2;
            }else if(nbrTailDir == 4){
                nbrMoveExpDir -= 2;
            }
            break;
        case 1:
            if(nbrTailDir == 2){
                nbrMoveExpDir += 2;
            }else if(nbrTailDir == 3){
                nbrMoveExpDir += 2;
            }
            break;

        case 2:
            if(nbrTailDir == 0){
                nbrMoveExpDir -= 2;
            }else if(nbrTailDir == 1){
                nbrMoveExpDir -= 2;
            }
            nbrMoveExpDir += 2;
            break;

        case 3:
            if(nbrTailDir == 1){
                nbrMoveExpDir -= 2;
            }else if(nbrTailDir == 5){
                nbrMoveExpDir += 2;
            }
            nbrMoveExpDir += 2;
            break;

        case 4:
            if(nbrTailDir == 0){
                nbrMoveExpDir += 2;
            }else if(nbrTailDir == 5){
                nbrMoveExpDir += 2;
            }
            nbrMoveExpDir += 2;
            break;

        case 5:
            if(nbrTailDir == 3){
                nbrMoveExpDir -= 2;
            }else if(nbrTailDir == 4){
                nbrMoveExpDir -= 2;
            }
            nbrMoveExpDir += 4;
            break;

        default: std::cout << "default\n"; // no error
                 break;
    }
    if(nbrMoveExpDir < 0){
        nbrMoveExpDir = 10 - abs(nbrMoveExpDir)%10;
    }else{
        nbrMoveExpDir = nbrMoveExpDir%10;
    }

    return nbrMoveExpDir;
}

int TestParticle::headMarkColor() const {
  switch(state) {
    case State::Leader:   return 0xff0000;
    case State::Follower: return 0x0000ff;
  }

  return -1;
}

int TestParticle::headMarkDir() const {
  if(state == State::Leader){
      return moveDir;
  }else{
      return moveDir;
  }

}

int TestParticle::tailMarkColor() const {
  return headMarkColor();
}

QString TestParticle::inspectionText() const {
  QString text;
  text += "Global Info:\n";
  text += "  head: (" + QString::number(head.x) + ", "
                      + QString::number(head.y) + ")\n";
  text += "  orientation: " + QString::number(orientation) + "\n";
  text += "  globalTailDir: " + QString::number(globalTailDir) + "\n\n";
  text += "Local Info:\n";
  text += "  state: ";
  text += [this](){
    switch(state) {
      case State::Leader:   return "leader\n";
      case State::Follower: return "follower\n";
      default:              return "no state\n";
    }
  }();
  text += "  moveDir: " + QString::number(moveDir) + "\n";

  return text;
}

TestParticle& TestParticle::nbrAtLabel(int label) const {
  return AmoebotParticle::nbrAtLabel<TestParticle>(label);
}

int TestParticle::labelOfFirstNbrInState(
    std::initializer_list<State> states, int startLabel) const {
  auto prop = [&](const TestParticle& p) {
    for (auto state : states) {
      if (p.state == state) {
        return true;
      }
    }
    return false;
  };

  return labelOfFirstNbrWithProperty<TestParticle>(prop, startLabel);
}


TestSystem::TestSystem() {
  // Insert the leader at (0,0) and the follower at (-1,0).
  insert(new TestParticle(Node(0, 0), -1, 0, *this,
                              TestParticle::State::Leader));
  insert(new TestParticle(Node(0, -1), -1, 0, *this,
                              TestParticle::State::Follower));
}

bool TestSystem::hasTerminated() const {
  #ifdef QT_DEBUG
    if (!isConnected(particles)) {
      return true;
    }
  #endif

  return false;
}
