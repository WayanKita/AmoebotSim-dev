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
        _moveDirExp(-1),
        _branchDir(-1),
        _branchDirExp(-1) {}

void FillBoundingBox::activate() {
    // if particle is contracted
    if (isContracted()) {

        // *** ROOT *** //
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

        // *** INACTIVE *** //
        else if (_state == State::Inactive) {
            //if particle is Inactive and has a follower or leader in its neighbourhood
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

        // *** FOLLOWER || COATER *** //
        else if (_state == State::Follower || _state == State::Coater) {
            if(hasNbrAtLabel(_moveDir)){
                FillBoundingBox& moveDirNbr = nbrAtLabel(_moveDir);
                if(moveDirNbr._state == State::Retired){
                    int label = getExpandLabel();
                    if(label >= 0){
                        expand(label);
                        _moveDir = label;
                        _state = State::Leader;
                    }else{
                        _state = State::Retired;
                    }
                    return;
                }else if(_state == State::Coater){
                    int rndNumber = randInt(1, 100);
                    if(rndNumber > 95 && !hasNbrInState({State::Root, State::Sroot})){
                        int label = getExpandLabel();
                        if(label > -1){
                            _branchDir = _moveDir;
                            expand(label);
                            _branchDirExp = updateLeaderDir(moveDirNbr);
                            _moveDir = label;
                            _state = State::Leader;
                        }
                        return;
                    }
                }
            }
            return;
        }

        // *** LEADER *** //
        else if (_state == State::Leader) {
            //if particle is a Leader, move in a free direction
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

        // *** BRANCH *** //
        else if (_state == State::Branch) {
            if(hasNbrAtLabel(_moveDir)){
                FillBoundingBox& particle = nbrAtLabel(_moveDir);
                if(particle._state == State::Retired){
                    if(_branchDir != -1){
                        if(hasNbrAtLabel(_branchDir)){
                            FillBoundingBox& particle_brch = nbrAtLabel(_branchDir);
                            if(particle_brch._state == State::Retired){
                                _state = State::Retired;
                            }
                        }
                    }else{
                        _state = State::Retired;
                    }

                }
            }
            return;
        }
        return;

    // ************    EXPANDED    ************ //
    } else {

        // *** SMALL ROOT *** //
        if (_state == State::Sroot) {
            if (getFollowerLabel() < 0){
                // bug leader expdir
                contractTail();
                _state = State::Root;
                return;
            }
            int label = getPullLabel();
            if(label > -1){
                FillBoundingBox& follower = nbrAtLabel(label);
                int moveDir = (tailDir() + 3) % 6;
                pull(label);
                if(follower._state == State::Follower){
                    _state = State::Root;
                }
                updateMoveDir(follower, moveDir);
            }
            return;
        }

        // *** SMALL BRANCH *** //
        else if (_state == State::Sbranch) {
            int label = getPullLabel();
            if(label > -1){
                FillBoundingBox& follower = nbrAtLabel(label);
                FillBoundingBox& moveBranchDirNbr = nbrAtLabel(_branchDirExp);
                int moveDir = (tailDir() + 3) % 6;
                //handleBranchPull(follower);
                pull(label);
                _state = State::Branch;
                updateMoveDir(follower, moveDir);
                updateBranchDir(moveBranchDirNbr);
            }
            return;
        }

        // *** BRANCH *** //
        else if (_state == State::Branch) {
            int label = getPullLabel();
            if(label > -1){
                FillBoundingBox& follower = nbrAtLabel(label);
                FillBoundingBox& moveDirNbr = nbrAtLabel(_branchDirExp);
                int moveDir = (tailDir() + 3) % 6;
                //handleBranchPull(follower);
                pull(label);
                follower._state = State::Sbranch;
                updateMoveDir(follower, moveDir);
                updateSbranchExpDir(follower, moveDirNbr);
                _state = State::Coater;
                _branchDir = -1;
                _branchDirExp = -1;
            }
            return;
        }

        // *** ROOT *** //
        else if (_state == State::Root) {
            int label = getPullLabel();
            if(label > -1){
                FillBoundingBox& follower = nbrAtLabel(label);
                int moveDir = (tailDir() + 3) % 6;
                pull(label);
                if(follower._state == State::Follower){
                    follower._state = State::Sroot;
                }
                if(_moveDirExp == -1){
                    _state = State::Leader;
                }else{
                    _state = State::Coater;
                }
                updateMoveDir(follower, moveDir);
                return;
            }
        }

        // *** LEADER *** //
        else if (_state == State::Leader) {
            // if particle is a Leader, pull one of its follower
            if(_branchDir == -1){
                // pulling coater
                int label = getPullLabel();
                if(label > -1){
                    int moveDir = (tailDir() + 3) % 6;
                    FillBoundingBox& follower = nbrAtLabel(label);
                    pull(label);
                    updateMoveDir(follower, moveDir);
                }
            }else{
                // pulling branch
                int label = getPullLabel();
                if(label > -1){
                    int moveDir = (tailDir() + 3) % 6;
                    FillBoundingBox& branch = nbrAtLabel(label);
                    if(branch._state == State::Branch){
                        // pull branch
                        //handleBranchPull(branch);
                        FillBoundingBox& otherFollower = branch.nbrAtLabel(branch._moveDir);
                        pull(label);
                        updateMoveDir(branch, moveDir);
                        updateBranchExpDir(branch, otherFollower);
                        _branchDir = -1;
                        _branchDirExp = -1;
                    }else{
                        // first pull, pull coater and transform it into an exp Sbranch
                        FillBoundingBox& otherFollower = nbrAtLabel(_branchDirExp);
                        pull(label);
                        branch._state = State::Sbranch;
                        branch._branchDir = dirToNbrDir(branch, _branchDir);
                        updateMoveDir(branch, moveDir);
                        updateSbranchExpDir(branch, otherFollower);
                    }
                }
                return;
            }

            return;

        // *** FOLLOWER || COATER *** //
        }else if(_state == State::Follower || _state == State::Coater){
            int label;
            int moveDir = (tailDir() + 3) % 6;

            if(hasNbrInState({State::Inactive})){
                return;
            }else{
                label = getFollowerLabel();
                if(label == -1){
                    contractTail();
                    return;
                }
            }
            label = getPullLabel();
            if(label > -1){
                FillBoundingBox& follower = nbrAtLabel(label);
                //handleBranchPull(follower);
                pull(label);
                updateMoveDir(follower, moveDir);
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
    case State::Root:       return 0xfdf421;
    case State::Sroot:      return 0xb1aa17;
    case State::Branch:     return 0x210808;
    case State::Sbranch:    return 0xffff00;
  }
  return -1;
}

int FillBoundingBox::tailMarkColor() const {
  return headMarkColor();
}

int FillBoundingBox::headMarkDir() const {
    return (_state == State::Leader || _state == State::Follower || _state == State::Root || _state == State::Sroot || _state == State::Coater || _state== State::Branch || _state== State::Sbranch) ? _moveDir : -1;
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
    case State::Root:       return "yellow\n";
    case State::Branch:     return "brown\n";
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

int FillBoundingBox::getPointDirBranch(FillBoundingBox& nbr) const {
    if(nbr.isContracted()){
        return nbr._branchDir;
    }else{
        return nbr._branchDirExp;
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

void FillBoundingBox::updateMoveDir(FillBoundingBox& follower, int moveDir) const {
    follower._moveDir = dirToNbrDir(follower, moveDir);
    for(int i = 0; i<10;i++){
        if(pointsAtMe(follower, i)){
            if(follower.isHeadLabel(i)){
                follower._moveDirExp = i;
            }
        }
    }
}

// branch pulls coater to make him a Sbranch
void FillBoundingBox::updateSbranchExpDir(FillBoundingBox& smallBranch, FillBoundingBox& otherFollower) const {
    for(int i = 0; i<10;i++){
        if(smallBranch.hasNbrAtLabel(i)){
            if(otherFollower.isContracted()){
                if(smallBranch.nbrNodeReachedViaLabel(i) == otherFollower.head){
                    smallBranch._branchDirExp = i;
                }
            }else{
                if(smallBranch.nbrNodeReachedViaLabel(i) == otherFollower.tail()){
                    smallBranch._branchDirExp = i;
                }
            }
        }
    }
}


//when particle pulls a contracted branch
void FillBoundingBox::updateBranchExpDir(FillBoundingBox& branch, FillBoundingBox& otherFollower) const {
    for(int i = 0; i < 10; i++){
        if(branch.hasNbrAtLabel(i)){
            if(otherFollower.isContracted()){
                if(branch.nbrNodeReachedViaLabel(i) == otherFollower.head){
                    branch._branchDirExp = i;
                }
            }else{
                if(branch.nbrNodeReachedViaLabel(i) == otherFollower.tail()){
                    branch._branchDirExp = i;
                }
            }
        }
    }
}

void FillBoundingBox::updateBranchDir(FillBoundingBox& moveBranchDirNbr) const {
    for(int i = 0; i < 6; i++){
        if(hasNbrAtLabel(i)){
            if(moveBranchDirNbr.isContracted()){
                if(nbrNodeReachedViaLabel(i) == moveBranchDirNbr.head){
                    moveBranchDirNbr._branchDir = i;
                }
            }else{
                if(nbrNodeReachedViaLabel(i) == moveBranchDirNbr.tail()){
                    moveBranchDirNbr._branchDir = i;
                }
            }
        }
    }
}


// when coater branches off to become leader, update new leader info
int FillBoundingBox::updateLeaderDir(FillBoundingBox& moveDirNbr) const {
    for(int i = 0; i < 10; i++){
        if(hasNbrAtLabel(i)){
            if(moveDirNbr.isContracted()){
                if(nbrNodeReachedViaLabel(i) == moveDirNbr.head){
                    return i;
                }
            }else{
                if(nbrNodeReachedViaLabel(i) == moveDirNbr.tail()){
                    return i;
                }
            }

        }
    }
    return -1;
}

void FillBoundingBox::inverseBranchDir(FillBoundingBox& branch) const {
    int moveDir = branch._branchDir;
    int moveDirExp = branch._branchDirExp;
    branch._branchDir = branch._moveDir;
    branch._branchDirExp = branch._moveDirExp;
    branch._moveDir = moveDir;
    branch._moveDirExp = moveDirExp;
}

bool FillBoundingBox::checkIfInverseBranchPull(FillBoundingBox& branch) const {
    if(branch._state != State::Branch && branch._state != State::Sbranch){
        return false;
    }
    if(branch._branchDir < 0){
        cout << "SHOULD NOT GET HERE";
        return false;
    }
    if(pointsAtMe(branch, branch._branchDir)){
        return true;
    }else if(pointsAtMe(branch, branch._moveDir)){
        return false;
    }
    return false;
}

void FillBoundingBox::handleBranchPull(FillBoundingBox& nbr) const {
    if(nbr._state == State::Branch){
        if(checkIfInverseBranchPull(nbr)){
            inverseBranchDir(nbr);
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
            if (getPointDir(follower) != -1 && (follower._state == State::Follower || follower._state == State::Sroot ||follower._state == State::Root ||follower._state == State::Coater || follower._state == State::Sbranch ||follower._state == State::Branch)){
                if(isContracted()){
                    if(pointsAtMe(follower, getPointDir(follower))){
                        return label;
                    }
                    if(follower._state == State::Branch || follower._state == State::Sbranch || follower._state == State::Leader){
                        if(pointsAtMe(follower, follower._branchDir)){
                            return label;
                        }
                    }
                }else{
                    if(pointsAtMyTail(follower, getPointDir(follower))){
                        if(isHeadLabel(label)){
                            label=(label+1)%10;
                        }
                        return label;
                    }
                    if(follower._state == State::Leader){
                        if(pointsAtMe(follower, follower._branchDirExp)){
                            return label;
                        }
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
            if (getPointDir(follower) != -1 && follower.isContracted() && (follower._state == State::Follower ||follower._state == State::Root ||follower._state == State::Coater || follower._state == State::Branch)){
                if(pointsAtMyTail(follower, getPointDir(follower))){
                    if(isHeadLabel(label)){
                        label=(label+1)%10;
                    }
                    return label;
                }
            }
            if (getPointDir(follower) != -1 && follower.isContracted() && follower._state == State::Branch){
                if(pointsAtMyTail(follower, follower._branchDir)){
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
  int num_row = 2;
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
