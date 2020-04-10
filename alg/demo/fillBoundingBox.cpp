/* Copyright (C) 2020 Joshua J. Daymude, Robert Gmyr, and Kristian Hinnenthal.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

#include "alg/demo/fillBoundingBox.h"
#include <iostream>
using namespace std;
using std::merge;


FillBoundingBox::FillBoundingBox(const Node head, const int globalTailDir,
                                     const int orientation,
                                     AmoebotSystem& system,
                                     State state)
    : AmoebotParticle(head, globalTailDir, orientation, system),
      _state(state),
      _moveDir(-1),
      _moveDirExp(-1) {}

void FillBoundingBox::activate() {
    // if particle is contracted
    if (isContracted()) {

        if (_state == State::Root) {
            if(_moveDir != -1){
                if (hasNbrAtLabel(_moveDir)) {
                    FillBoundingBox& particle = nbrAtLabel(_moveDir);
                    if(particle._state == State::Retired){
                        return;
                    }
                    if(particle._state == State::Leader || particle._state == State::Coater){
                        return;
                    }
                }
            }
            if(!hasNbrInState({State::Inactive})){
                int label_follower = getFollowerLabel();
                int label_expand = getExpandLabel();
                if(label_follower > -1){
                    if(label_expand > -1){
                        expand(label_expand);
                        _moveDir = label_expand;
                        return;
                    }
                }
            }
            return;
        }

        //if particle is Inactive and has a follower or leader in its neighbourhood
        if (_state == State::Inactive) {
            if (hasNbrInState({State::Leader, State::Follower, State::Root})) {
                // if particle is Innactive, attempt to follow leader, else follow follower
                if (hasNbrInState({State::Leader, State::Root})){
                    _state = State::Follower;
                    _moveDir = labelOfFirstNbrInState({State::Leader, State::Root});
                    return;
                }else{
                    _state = State::Follower;
                    _moveDir = labelOfFirstNbrInState({State::Follower});
                    return;
                }
            }
        }
        else if (_state == State::Follower || _state == State::Coater) {
            if(hasNbrAtLabel(_moveDir)){
                FillBoundingBox& particle = nbrAtLabel(_moveDir);
                if(particle._state == State::Retired){
                    int label = getExpandLabel();
                    if(label >= 0){
                        expand(label);
                        _moveDir = label;
                        _state = State::Leader;
                    }else{
                        _state = State::Retired;
                    }
                    return;
                }
            }
            return;
        } //Follower contracted subroutine end

        //if particle is a Leader, move in a free direction
        else if (_state == State::Leader) {
            if(!hasNbrInState({State::Inactive})){
                int label = getExpandLabel();
                if(label >= 0 ){
                    _moveDir = label;
                    expand(label);
                    return;
                }
                _state = State::Retired;
                return;
            }
            return;
        }
        return;
    } else {  // isExpanded().

        if (_state == State::Sroot) {
            if (getFollowerLabel() < 0){
                contractTail();
                _state = State::Root;
                return;
            }
            int label = getPullLabel();
            if(label > -1){
                FillBoundingBox& follower = nbrAtLabel(label);
                int contractDir = (tailDir() + 3) % 6;
                pull(label);
                if(follower._state == State::Follower){
                    _state = State::Root;
                }
                follower._moveDir = dirToNbrDir(follower, contractDir);
                updateMoveDirExp(follower);
                return;
            }
        }
        else if (_state == State::Root) {
//            for (int label = 0; label < 10; label++){
//                if(hasNbrAtLabel(label)){
//                    FillBoundingBox& follower = nbrAtLabel(label);
//                    if(follower._state == State::Leader){
//                        continue;
//                    }
//                    if(pointsAtMyTail(follower, getPointDir(follower))){
//                        if (follower.isContracted() && (follower._state == State::Follower ||follower._state == State::Root)) {
//                            if(canPull(label)){
//                                int contractDir = (tailDir() + 3) % 6;
//                                if(isHeadLabel(label)){
//                                    label=(label+1)%10;
//                                }
//                                pull(label);
//                                if(follower._state == State::Follower){
//                                    follower._state = State::Sroot;
//                                }
//                                if(_moveDirExp == -1){
//                                    _state = State::Leader;
//                                }else{
//                                    _state = State::Coater;
//                                }
//                                follower._moveDir = dirToNbrDir(follower, contractDir);
//                                updateMoveDirExp(follower);
//                                return;
//                            }
//                        }

//                    }
//                }

//            }
            int label = getPullLabel();
            if(label > -1){
                FillBoundingBox& follower = nbrAtLabel(label);
                int contractDir = (tailDir() + 3) % 6;
                pull(label);
                if(follower._state == State::Follower){
                    follower._state = State::Sroot;
                }
                if(_moveDirExp == -1){
                    _state = State::Leader;
                }else{
                    _state = State::Coater;
                }
                follower._moveDir = dirToNbrDir(follower, contractDir);
                updateMoveDirExp(follower);
                return;
            }

        }

        // if particle is a Leader, pull one of its follower
        else if (_state == State::Leader) {
            for (int label = 0; label < 10; label++){
                if(hasNbrAtLabel(label)){
                    FillBoundingBox& follower = nbrAtLabel(label);
                    if(pointsAtMyTail(follower, getPointDir(follower))){
                        if (follower.isContracted() && (follower._state == State::Follower ||follower._state == State::Root ||follower._state == State::Coater)) {
                            if(canPull(label)){
                                if(isHeadLabel(label)){
                                    label=(label+1)%10;
                                }
                                int contractDir = (tailDir() + 3) % 6;
                                pull(label);
                                follower._moveDir = dirToNbrDir(follower, contractDir);
                                updateMoveDirExp(follower);
                                return;
                            }
                        }
                    }
                }
            }
            return;

        // if particle is Follower
        }else if(_state == State::Follower || _state == State::Coater){
            if(hasNbrInState({State::Inactive})){
                return;
            }else{
                int label = getFollowerLabel();
                if(label == -1){
                    contractTail();
                    return;
                }
            }
            for (int label = 0; label < 10; label++){
                if (hasNbrAtLabel(label)){
                    FillBoundingBox& follower = nbrAtLabel(label);
                    if(follower._state == State::Follower ||follower._state == State::Root ||follower._state == State::Coater){
                        if(pointsAtMyTail(follower, getPointDir(follower))){
                            if (follower.isContracted()) {
                                if(canPull(label)){
                                    int contractDir = (tailDir() + 3) % 6;
                                    if(isHeadLabel(label)){
                                        label=(label+1)%10;
                                    }
                                    pull(label);
                                    follower._moveDir = dirToNbrDir(follower, contractDir);
                                    updateMoveDirExp(follower);
                                    return;
                                }
                            }
                        }
                    }
                }
            }
            return;

        }
        return;
    }
    return;
}

int FillBoundingBox::headMarkColor() const {
  switch(_state) {
    case State::Leader:     return 0xff0000;
    case State::Inactive:   return 0x5C5C5C;
    case State::Retired:    return 0x000000;
    case State::Leaf:       return 0x00ff00;
    case State::Follower:   return 0x0000ff;
    case State::Coater:     return 0x4c4cff;
    case State::Branch:     return 0xfd8521;
    case State::Root:       return 0xfdf421;
    case State::Sroot:      return 0xb1aa17;
  }
  return -1;
}

int FillBoundingBox::tailMarkColor() const {
  return headMarkColor();
}

int FillBoundingBox::headMarkDir() const {
  return (_state == State::Leader || _state == State::Follower || _state == State::Root || _state == State::Sroot || _state == State::Coater) ? _moveDir : -1;
}

FillBoundingBox& FillBoundingBox::nbrAtLabel(int label) const {
  return AmoebotParticle::nbrAtLabel<FillBoundingBox>(label);
}

QString FillBoundingBox::inspectionText() const {
  QString text;
  text += "Global Info:\n";
  text += "  head: (" + QString::number(head.x) + ", "
                      + QString::number(head.y) + ")\n";
  text += "  orientation: " + QString::number(orientation) + "\n";
  text += "  globalTailDir: " + QString::number(globalTailDir) + "\n\n";
  text += "Local Info:\n";
  text += "  state: ";
  text += [this](){
    switch(_state) {
    case State::Leader:     return "red\n";
    case State::Inactive:   return "grey\n";
    case State::Retired:    return "black\n";
    case State::Leaf:       return "green\n";
    case State::Follower:   return "blue\n";
    case State::Root:   return "yellow\n";
    case State::Branch:   return "orange\n";
    }
    return "no state\n";
  }();

  return text;
}

FillBoundingBox::State FillBoundingBox::getRandColor() const {
  // Randomly select an integer and return the corresponding state via casting.
  return static_cast<State>(randInt(0, 7));
}

int FillBoundingBox::getPointDir(FillBoundingBox& nbr) const {
    if(nbr.isContracted()){
        return nbr._moveDir;
    }else{
        return nbr._moveDirExp;
    }
}


int FillBoundingBox::getExpandLabel() const {
    for (int label = 0; label < 6; label++){
        if(canExpand(label)){
            while(true){
                int label = randInt(0, 6); //move in random location
                if(canExpand(label)){
                    return label;
                }
            }

        }
    }
    return -1;
}

void FillBoundingBox::updateMoveDirExp(FillBoundingBox& follower) const {
    for(int i = 0; i<10;i++){
        if(pointsAtMe(follower, i)){
            if(follower.isHeadLabel(i)){
                follower._moveDirExp = i;
            }
        }
    }
}


int FillBoundingBox::getFollowerLabel() const {
    int label_limit;
    if(isContracted()){
        label_limit = 6;
    }else{
        label_limit = 10;
    }
    for (int label = 0; label < label_limit; label++){
        if (hasNbrAtLabel(label)){
            FillBoundingBox& follower = nbrAtLabel(label);
            if (getPointDir(follower) != -1 && (follower._state == State::Follower || follower._state == State::Sroot ||follower._state == State::Root ||follower._state == State::Coater)){
                if(isContracted()){
                    if(pointsAtMe(follower, getPointDir(follower))){
                        return label;
                    }
                }else{
                    if(pointsAtMyTail(follower, getPointDir(follower))){
                        if(isHeadLabel(label)){
                            label=(label+1)%10;
                        }
                        return label;
                    }
                }
            }
        }
    }
    return -1;
}


int FillBoundingBox::getPullLabel() const {
    int label_limit = 10;
    for (int label = 0; label < label_limit; label++){
        if (hasNbrAtLabel(label)){
            FillBoundingBox& follower = nbrAtLabel(label);
            if (getPointDir(follower) != -1 && follower.isContracted() && (follower._state == State::Follower ||follower._state == State::Root ||follower._state == State::Coater)){
                if(pointsAtMyTail(follower, getPointDir(follower))){
                    if(isHeadLabel(label)){
                        label=(label+1)%10;
                    }
                    return label;
                }
            }
        }
    }
    return -1;
}

int FillBoundingBox::labelOfFirstNbrInState(
    std::initializer_list<State> states, int startLabel) const {
  auto prop = [&](const FillBoundingBox& p) {
    for (auto state : states) {
      if (p._state == state) {
          return true;
      }
    }
    return false;
  };

  return labelOfFirstNbrWithProperty<FillBoundingBox>(prop, startLabel);
}


std::set<int>  FillBoundingBox::findFollowerChildren() const {
    std::set<int> children;
    for (int label = 0; label < 6; label++) {
        if (hasNbrAtLabel(label)) {
        FillBoundingBox& particle = nbrAtLabel(label);
            if (pointsAtMe(particle, getPointDir(particle))){
                children.insert(label);
            }
        }
    }
    return children;
}

bool FillBoundingBox::hasNbrInState(std::initializer_list<State> states)const {
    return labelOfFirstNbrInState(states) != -1;
}



FillBoundingBoxSystem::FillBoundingBoxSystem(unsigned int numParticles) {

  // In order to enclose an area that's roughly 3.7x the # of particles using a
  // regular hexagon, the hexagon should have side length 1.4*sqrt(# particles).
  int sideLen = static_cast<int>(std::round(1.4 * std::sqrt(numParticles)));
  Node boundNode(0, 0);
  for (int dir = 0; dir < 6; ++dir) {
    for (int i = 0; i < sideLen; ++i) {
        if (dir == 5 && i == sideLen-1){
            break;
        }else{
            insert(new Object(boundNode));
            boundNode = boundNode.nodeInDir(dir);
        }
    }
  }



  // Let s be the bounding hexagon side length. When the hexagon is created as
  // above, the nodes (x,y) strictly within the hexagon have (i) -s < x < s,
  // (ii) 0 < y < 2s, and (iii) 0 < x+y < 2s. Choose interior nodes at random to
  // place particles, ensuring at most one particle is placed at each node.
  std::set<Node> occupied;
  Node node(-1, 1);
  insert(new FillBoundingBox(node, -1, randDir(), *this, FillBoundingBox::State::Root));
  occupied.insert(node);

  int cond = 0;
  int num_particle_row = 8;
  int num_row = 22;
  for (int y = 0; y < num_row; ++y) {
  cond = 8+y;
    for (int x = 0; x < num_particle_row; ++x) {
        if (x == cond) {
            cond++;
            continue;
        }else{
            Node node(0-x-2+y, 0-y+1);
            insert(new FillBoundingBox(node, -1, randDir(), *this, FillBoundingBox::State::Inactive));
            occupied.insert(node);
        }

    }
  }
}
