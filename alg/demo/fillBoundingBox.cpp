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
        _branchDirExp(-1),
        _compareInt(0){}

void FillBoundingBox::activate() {
    // if particle is contracted
    if (isContracted()) {

        // *** LEADER *** //
        if (_state == State::Leader) {
            if(_moveDir != -1){
                if (hasNbrAtLabel(_moveDir)) {
                    FillBoundingBox& particle = nbrAtLabel(_moveDir);
                    if(particle._state == State::Retired){
                        _state = State::Finished;
                        return;
                    }
                    if(particle._state == State::Leaf || particle._state == State::Filler || particle._state == State::Branch || particle._state == State::Sbranch){
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
            //if particle is Inactive and has a follower or leaf in its neighbourhood
            if (hasNbrInState({State::Leaf, State::Follower, State::Leader, State::SmallLeader})) {
                // if particle is Innactive, attempt to follow leaf, else follow follower
                if (hasNbrInState({State::Leaf, State::Leader, State::SmallLeader})){
                    _state = State::Follower;
                    _moveDir = labelOfFirstNbrInState({State::Leaf, State::Leader, State::SmallLeader});
                    return;
                }else{
                    _state = State::Follower;
                    _moveDir = labelOfFirstNbrInState({State::Follower});
                    return;
                }
            }
        }

        // *** FOLLOWER || FILLER *** //
        else if (_state == State::Follower || _state == State::Filler) {
            if(hasNbrAtLabel(_moveDir)){
                FillBoundingBox& moveDirNbr = nbrAtLabel(_moveDir);
                if(moveDirNbr._state == State::Retired){
                    int label = getExpandLabel();
                    if(label >= 0){
                        expand(label);
                        _moveDir = label;
                        _state = State::Leaf;
                    }else{
                        _state = State::Retired;
                        system.getCount("# Retired").record();
                    }
                }else if(_state == State::Filler){
                    int rndNumber = randInt(1, 100);
                    if(true && !hasNbrInState({State::Leader, State::SmallLeader, State::Sbranch, State::Branch, State::Leaf})){
                        int label = getExpandLabel();
                        if(label > -1){
                            expand(label);
                            _branchDir = _moveDir;
                            _branchDirExp = (findExpDir(label, tailDir()) + 2 + findCCDistance(label,_moveDir))%10;
                            _moveDir = label;
                            _state = State::Leaf;
                        }
                    }
                }
            }
            return;
        }

        // *** LEAF *** //
        else if (_state == State::Leaf) {
            //if particle is a Leaf, move in a free direction
            if(!hasNbrInState({State::Inactive})){
                int label = getExpandLabel();
                if(label >= 0 ){
                    _moveDir = label;
                    expand(label);
                    return;
                }
                _state = State::Retired;
                system.getCount("# Retired").record();
                return;
            }
            return;
        }

        // *** BRANCH *** //
        else if (_state == State::Branch) {
//            if(hasNbrAtLabel(_moveDir)){
//                FillBoundingBox& particle = nbrAtLabel(_moveDir);
//                if(particle._state == State::Retired){
//                    if(_branchDir != -1){
//                        if(hasNbrAtLabel(_branchDir)){
//                            FillBoundingBox& particle_brch = nbrAtLabel(_branchDir);
//                            if(particle_brch._state == State::Retired){
//                                _state = State::Retired;
//                            }
//                        }
//                    }else{
//                        _state = State::Retired;
//                    }

//                }
//            }
//            return;
            if(hasNbrAtLabel(_branchDir)){
                FillBoundingBox& particle = nbrAtLabel(_branchDir);
                if(particle._state == State::Retired){
                    _state = State::Filler;
                    _branchDir = -1;
                    _branchDirExp = -1;
                    return;
                }
            }else if(hasNbrAtLabel(_moveDir)){
                FillBoundingBox& particle = nbrAtLabel(_moveDir);
                if(particle._state == State::Retired){
                    _state = State::Filler;
                    _moveDir = _branchDir;
                    _moveDirExp = _branchDirExp;
                    return;
                }
            }
            return;

        }
        return;

    // ************    EXPANDED    ************ //
    } else {

        // *** SMALL LEADER *** //
        if (_state == State::SmallLeader) {
            if (getFollowerLabel() < 0){
                if(!hasNbrInState({State::Inactive})){
                    contractTail();
                    _state = State::Leader;
                    return;
                }
            }
            int label = getPullLabel();
            if(label > -1){
                FillBoundingBox& follower = nbrAtLabel(label);
                int moveDir = (tailDir() + 3) % 6;
                pull(label);
                if(follower._state == State::Follower){
                    _state = State::Leader;
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
                handleBranchPull(follower);
                int moveDir = (tailDir() + 3) % 6;
                pull(label);
                _state = State::Branch;
                updateMoveDir(follower, moveDir);
            }
            return;
        }

        // *** BRANCH *** //
        else if (_state == State::Branch) {
            int label = getPullLabel();
            if(label > -1){
                FillBoundingBox& follower = nbrAtLabel(label);
                handleBranchPull(follower);
                int moveDir = (tailDir() + 3) % 6;
                pull(label);
                follower._state = State::Sbranch;
                updateMoveDir(follower, moveDir);
                follower._branchDir = _branchDir;
                follower._branchDirExp = findExpDir(follower._branchDir, follower.tailDir());
                _state = State::Filler;
                _branchDir = -1;
                _branchDirExp = -1;
            }
            return;
        }

        // *** LEADER *** //
        else if (_state == State::Leader) {
            int label = getPullLabel();
            if(label > -1){
                FillBoundingBox& follower = nbrAtLabel(label);
                int moveDir = (tailDir() + 3) % 6;
                pull(label);
                if(follower._state == State::Follower){
                    follower._state = State::SmallLeader;
                }
                if(_moveDirExp == -1){
                    _state = State::Leaf;
                }else{
                    _state = State::Filler;
                }
                updateMoveDir(follower, moveDir);
                return;
            }
        }

        // *** Leaf *** //
        else if (_state == State::Leaf) {
            // if particle is a Leaf, pull one of its follower
            // pulling branch
            int label = getPullLabel();
            if(label > -1){
                int moveDir = (tailDir() + 3) % 6;
                FillBoundingBox& branch = nbrAtLabel(label);
                if(branch._state == State::Branch){
                    // pull branch
                    handleBranchPull(branch);
                    pull(label);

                    updateMoveDir(branch, moveDir);
                    branch._branchDirExp = (findExpDir(branch._moveDir, branch.tailDir()) + 2+findCCDistance(branch._moveDir, branch._branchDir))%10;
                    _branchDir = -1;
                    _branchDirExp = -1;
                    return;
                }else if ((branch._state == State::Filler || branch._state == State::Leader) && _branchDirExp == -1){
                    int label = getPullLabel();
                    if(label > -1){
                        int moveDir = (tailDir() + 3) % 6;
                        FillBoundingBox& follower = nbrAtLabel(label);
                        pull(label);
                        updateMoveDir(follower, moveDir);
                    }
                }else{
                    // first pull, pull filler and transform it into an exp Sbranch
                    pull(label);
                    branch._state = State::Sbranch;
                    updateMoveDir(branch, moveDir);
                    branch._branchDir = _branchDir;
                    branch._branchDirExp = findExpDir(branch._branchDir, branch.tailDir());
                    _branchDir = -1;
                    _moveDir = -1;
                    return;
                }
            }

            return;

        // *** FOLLOWER || FILLER *** //
        }else if(_state == State::Follower || _state == State::Filler){
            int label;
            int moveDir = (tailDir() + 3) % 6;
            if(hasNbrInState({State::Inactive})){
                return;
            }else{
                if(_state == State::Follower){
                    label = getFollowerLabel();
                    if(label < 0){
                        contractTail();
                        return;
                    }
                }
            }
            label = getPullLabel();
            if(label > -1){
                FillBoundingBox& follower = nbrAtLabel(label);
                if(follower._state == State::Branch){
                    handleBranchPull(follower);
                    pull(label);
                    updateMoveDir(follower, moveDir);
                    follower._branchDirExp = (findExpDir(follower._moveDir, follower.tailDir()) + 2+findCCDistance(follower._moveDir, follower._branchDir))%10;
                }else{
                    pull(label);
                    updateMoveDir(follower, moveDir);
                }

            }
            return;
        }
        return;
    }
    return;
}





// **********     DEFAULT FUNCTIONS     **********//

int FillBoundingBox::headMarkColor() const {
  switch(_state) {
    case State::Leaf:       return 0x33a02c;
    case State::Inactive:   return 0xcab2d6;
    case State::Retired:    return 0x6a3d9a;
    case State::Follower:   return 0x1f78b4;
    case State::Filler:     return 0xa6cee3;
    case State::Leader:     return 0xff7f00;
    case State::SmallLeader:return 0xFF9224;
    case State::Branch:     return 0x583B23;
    case State::Sbranch:    return 0x7F5533;
  }
  return -1;
}

int FillBoundingBox::tailMarkColor() const {
  return headMarkColor();
}

int FillBoundingBox::headMarkDir() const {
    return (_state == State::Leaf || _state == State::Follower || _state == State::Leader || _state == State::SmallLeader || _state == State::Filler || _state== State::Branch || _state== State::Sbranch) ? _moveDir : -1;
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
    case State::Inactive:   return "grey\n";
    case State::Retired:    return "black\n";
    case State::Leaf:       return "green\n";
    case State::Follower:   return "blue\n";
    case State::Leader:       return "yellow\n";
    case State::Branch:     return "brown\n";
    case State::Filler:     return "Light Blue\n";
    case State::SmallLeader:      return "Dark Yellow\n";
    case State::Sbranch:    return "Yellow\n";
    }
    return "no state\n";
  }();

  return text;
}


// **********     CUSTOM HELPER FUNCTIONS     **********//


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
//    for(int i = 0; i<10;i++){
//        if(pointsAtMe(follower, i)){
//            if(follower.isHeadLabel(i)){
//                follower._moveDirExp = i;
//            }
//        }
//    }
    follower._moveDirExp = findExpDir(follower._moveDir, follower.tailDir());
}

// branch pulls filler to make him a Sbranch
void FillBoundingBox::updateSbranchExpDir(FillBoundingBox& smallBranch, FillBoundingBox& otherFollower) const {
    for(int i = 0; i<10;i++){
        if(smallBranch.hasNbrAtLabel(i)){
            FillBoundingBox& compareNbr = smallBranch.nbrAtLabel(i);
            if(compareNbr._compareInt == 1){
                smallBranch._branchDirExp = i;
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
            }

        }
    }
}

int FillBoundingBox::updateBranchDir(FillBoundingBox& moveBranchDirNbr) const {
    for(int i = 0; i < 6; i++){
        if(hasNbrAtLabel(i)){
            FillBoundingBox& compareNbr = nbrAtLabel(i);
            if(compareNbr._compareInt == 1){
                return i;
            }
        }
    }
}


// when filler branches off to become leaf, update new leaf info
int FillBoundingBox::updateLeafDir(FillBoundingBox& moveDirNbr) const {
    for(int i = 0; i < 10; i++){
        if(hasNbrAtLabel(i)){
            FillBoundingBox& compareNbr = nbrAtLabel(i);
            if(compareNbr._compareInt == 1){
                return i;
            }
        }
    }
    return -1;
}


int FillBoundingBox::getBranchDir() const {
    if(isContracted()){
        for(int i = 0; i < 6; i++){
            if(hasNbrAtLabel(i)){
                FillBoundingBox& compareNbr = nbrAtLabel(i);
                if(compareNbr._compareInt == 1){
                    return i;
                }
            }
        }
    }else{
        for(int i = 0; i < 10; i++){
            if(hasNbrAtLabel(i)){
                FillBoundingBox& compareNbr = nbrAtLabel(i);
                if(compareNbr._compareInt == 1){
                    return i;
                }
            }
        }
    }

    return -1;
}

int FillBoundingBox::findCCDistance(int begin, int end){
    for(int distance = 0; distance < 6; distance++){
        if(begin == end){
            return distance;
        }
        begin++;
        begin = begin % 6;
    }
    return -1;
}

int FillBoundingBox::findExpDir(int moveDir, int taildir) const{
    return calculateMoveExpDir(moveDir, (taildir + 3) % 6);
}

void FillBoundingBox::setBranchDir(FillBoundingBox& particle) const {
    if(isContracted()){
        for(int i = 0; i < 6; i++){
            if(particle.hasNbrAtLabel(i)){
                FillBoundingBox& compareNbr = particle.nbrAtLabel(i);
                if(compareNbr._compareInt == 1){
                    particle._branchDir = i;
                }
            }
        }
    }else{
        for(int i = 0; i < 10; i++){
            if(particle.hasNbrAtLabel(i)){
                FillBoundingBox& compareNbr = particle.nbrAtLabel(i);
                if(compareNbr._compareInt == 1){
                    particle._branchDirExp = i;
                }
            }
        }
    }
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



int FillBoundingBox::calculateMoveExpDir(int tailDir, int nbrTailDir) const{
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
            if ((follower._state == State::Follower || follower._state == State::SmallLeader ||follower._state == State::Leader ||follower._state == State::Filler || follower._state == State::Sbranch || follower._state == State::Branch || follower._state == State::Leaf)){
                if(isContracted()){
                    if(getPointDir(follower) != -1){
                        if(pointsAtMe(follower, getPointDir(follower))){
                            return label;
                        }
                    }

                    if(getPointDirBranch(follower) != -1 && (follower._state == State::Branch || follower._state == State::Sbranch || follower._state == State::Leaf)){
                        if(pointsAtMe(follower, getPointDirBranch(follower))){
                            return label;
                        }
                    }
                }else{
                    if(getPointDir(follower) != -1){
                        if(pointsAtMyTail(follower, getPointDir(follower))){
                            if(isHeadLabel(label)){
                                label=(label+1)%10;
                            }
                            return label;
                        }
                    }
                    if(getPointDirBranch(follower) != -1 && (follower._state == State::Leaf || follower._state == State::Branch || follower._state == State::Sbranch)){
                        if(pointsAtMyTail(follower, getPointDirBranch(follower))){
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
            if (getPointDir(follower) != -1 && follower.isContracted() && (follower._state == State::Follower ||follower._state == State::Leader ||follower._state == State::Filler || follower._state == State::Branch)){
                if(pointsAtMyTail(follower, getPointDir(follower))){
                    if(isHeadLabel(label)){
                        label=(label+1)%10;
                    }
                    return label;
                }
            }
            if (getPointDirBranch(follower) != -1 && follower.isContracted() && follower._state == State::Branch){
                if(pointsAtMyTail(follower, getPointDirBranch(follower))){
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

bool FillBoundingBoxSystem::hasTerminated() const{
    for (auto p : particles) {
         auto tdp = dynamic_cast<FillBoundingBox*>(p);
        if(tdp->_state == FillBoundingBox::State::Finished ){
            return true;
        }
    }
    return false;
}

float FillBoundingBoxSystem::area(int x1, int y1, int x2, int y2, int x3, int y3)
{
   return abs((x1*(y2-y3) + x2*(y3-y1)+ x3*(y1-y2))/2.0);
}

bool FillBoundingBoxSystem::isInside(int x2, int y2, int x3, int y3, int x, int y)
{
   /* Calculate area of triangle ABC */
   float A = area (0, 0, x2, y2, x3, y3);

   /* Calculate area of triangle PBC */
   float A1 = area (x, y, x2, y2, x3, y3);

   /* Calculate area of triangle PAC */
   float A2 = area (0, 0, x, y, x3, y3);

   /* Calculate area of triangle PAB */
   float A3 = area (0, 0, x2, y2, x, y);

   /* Check if sum of A1, A2 and A3 is same as A */
   return (A == A1 + A2 + A3);
}

bool FillBoundingBoxSystem::checkIfNodeValid(Node node, int quadrant, int sideLength){
    switch (quadrant){
        case 0:
//            if(node.x >= 0 && node.x <= sideLength-node.y && node.y >=0 && node.y <= sideLength-node.x){
//                return true;
//            }
            return isInside(sideLength, 0, 0, sideLength, node.x, node.y);
            break;
        case 1:
//            if(node.x >= -sideLength && node.x <= 0 && node.y >= abs(node.x) && node.y <= sideLength){
//                return true;
//            }
            return isInside(0, sideLength, -sideLength, sideLength, node.x, node.y);
            break;
        case 2:
//            if(node.x >= -sideLength && node.x <= 0 && node.y >= 0 && node.y <= abs(node.x)){
//                return true;
//            }
            return isInside(-sideLength, sideLength, -sideLength, 0, node.x, node.y);
            break;
        case 3:
//            if(node.x <= 0 && node.x >= -(sideLength+node.y) && node.y <=0 && node.y >= -(sideLength+node.x)){
//                return true;
//            }
            return isInside(-sideLength, 0, 0, -sideLength, node.x, node.y);
            break;
        case 4:
//            if(node.x <= sideLength && node.x >= 0 && node.y <= abs(node.x) && node.y >= -sideLength){
//                return true;
//            }
            return isInside(0, -sideLength, sideLength, -sideLength, node.x, node.y);
            break;
        case 5:
            return isInside(sideLength, -sideLength, sideLength, 0, node.x, node.y);
            break;
    }
   return false;
}

bool FillBoundingBoxSystem::checkIfNodeDoubleValid(Node node, int quadrant){

}

bool FillBoundingBoxSystem::detectEdgeCase(Node node, int quadrant){
    switch (quadrant){
        case 0:
            return (node.x == 0);
            break;
        case 1:
            return (node.x == -node.y);
            break;
        case 2:
            return (node.y == 0);
            break;
        case 3:
            return (node.x == 0);
            break;
        case 4:
            return (node.y == -node.x);
            break;
        case 5:
            return (node.y == 0);
            break;
    }
   return false;
}


FillBoundingBoxSystem::FillBoundingBoxSystem(unsigned int sideLength, unsigned int objectShapeInt, unsigned int particleConfigInt) {
    _counts.push_back(new Count("# Retired"));
  // In order to enclose an area that's roughly 3.7x the # of particles using a
  // regular hexagon, the hexagon should have side length 1.4*sqrt(# particles).
  int sideLen = static_cast<int>(sideLength);

//  sideLen = 2;

  enum class ObjectShape {
      Hexagon,
      Line,
      Random,
      Random_10_1,
      Random_10_2,
      Random_10_3,
      Random_10_4,
      Random_10_5,
      Random_20_1,
      Random_20_2,
      Random_20_3,
      Random_20_4,
      Random_20_5,
      Random_Flat_1,
      Random_Flat_2,
      Random_Flat_3,
      Random_Flat_4,
      Random_Flat_5,
      Random_Flat_Generic,
      Cross,
      Test,
  };
  int objectInt = static_cast<int>(objectShapeInt);
  ObjectShape objectShape;
  switch(objectInt) {
     case 0  :
        objectShape = ObjectShape::Hexagon;
        break;
     case 1  :
        objectShape = ObjectShape::Line;
        break;
    case 10  :
        objectShape = ObjectShape::Random;
        break;
      case 11  :
          objectShape = ObjectShape::Random_10_1;
          break;
      case 12  :
          objectShape = ObjectShape::Random_10_2;
          break;
      case 13  :
          objectShape = ObjectShape::Random_10_3;
          break;
      case 14  :
          objectShape = ObjectShape::Random_10_4;
          break;
      case 15  :
          objectShape = ObjectShape::Random_10_5;
          break;
      case 21  :
          objectShape = ObjectShape::Random_20_1;
          break;
      case 22  :
          objectShape = ObjectShape::Random_20_2;
          break;
      case 23  :
          objectShape = ObjectShape::Random_20_3;
          break;
      case 24  :
          objectShape = ObjectShape::Random_20_4;
          break;
      case 25  :
          objectShape = ObjectShape::Random_20_5;
          break;
      case 31  :
          objectShape = ObjectShape::Random_Flat_1;
          break;
      case 32  :
          objectShape = ObjectShape::Random_Flat_2;
          break;
      case 33  :
          objectShape = ObjectShape::Random_Flat_3;
          break;
      case 34  :
          objectShape = ObjectShape::Random_Flat_4;
          break;
      case 35  :
          objectShape = ObjectShape::Random_Flat_5;
          break;
        case 39  :
          objectShape = ObjectShape::Random_Flat_Generic;
          break;
      case 40:
          objectShape = ObjectShape::Cross;
          break;
    case 99  :
        objectShape = ObjectShape::Test;
        break;
    default:
      objectShape = ObjectShape::Hexagon;

  }
  int numberParticles = 1;
  for (int i = (sideLen-1); i > 0; i--){
      numberParticles += i*6;
  }

  if(objectShape == ObjectShape::Line){
      Node boundNode(0, 0);
      for (int i = 0; i <= numberParticles; ++i) {
        insert(new Object(boundNode));
        boundNode = boundNode.nodeInDir(1);
      }
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(3);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(4);
      for (int i = 0; i <= numberParticles-1; ++i) {
              insert(new Object(boundNode));
              boundNode = boundNode.nodeInDir(4);
      }
  }else if (objectShape == ObjectShape::Hexagon){
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
  }else if (objectShape == ObjectShape::Test){
        numberParticles = 3;
          Node boundNode(0, 0);
          for (int i = 0; i <= numberParticles; ++i) {
            insert(new Object(boundNode));
            boundNode = boundNode.nodeInDir(1);
          }
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(2);
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(3);
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(4);
          for (int i = 0; i <= numberParticles; ++i) {
                  insert(new Object(boundNode));
                  boundNode = boundNode.nodeInDir(4);
          }
  }else if(objectShape == ObjectShape::Random){
      std::vector<int> objectPath;
      int quadrant = 0;
      int rndDir = 0;
      int prevRndDir = 3;
      Node boundNode(sideLength,0);
      boundNode = boundNode.nodeInDir(2);
      objectPath.push_back(2);
      insert(new Object(boundNode));
      int avDir [4] = {0,1,2,3};
      while(quadrant != 6){
//        for(int i = 0; i< 10;i++){
          rndDir = avDir[randInt(0,4)];
          if(!detectEdgeCase(boundNode.nodeInDir(rndDir), quadrant)){
              if(checkIfNodeValid(boundNode.nodeInDir(rndDir), quadrant, sideLength) && (prevRndDir+3)%6 != rndDir){
                  prevRndDir = rndDir;
                  boundNode = boundNode.nodeInDir(rndDir);
                  objectPath.push_back(rndDir);
                  insert(new Object(boundNode));
                  if(rndDir == avDir[0] || rndDir == avDir[3]){
                      if(quadrant != 5 && randInt(0,1) >= 0){
                          if(checkIfNodeValid(boundNode.nodeInDir(rndDir), quadrant, sideLength)){
                              boundNode = boundNode.nodeInDir(rndDir);
                              objectPath.push_back(rndDir);
                              insert(new Object(boundNode));
                          }
                      }
                  }
              }
          }else{
              if(quadrant == 5){
                  while(boundNode.x < sideLen){
                      boundNode = boundNode.nodeInDir(0);
                      objectPath.push_back(0);
                      insert(new Object(boundNode));
                  }
                  quadrant+=1;
                  break;
              }
              boundNode = boundNode.nodeInDir(rndDir);
              objectPath.push_back(rndDir);
              insert(new Object(boundNode));

              quadrant+=1;
              for(int i = 0; i < 4 ; i++ ){
                  avDir[i] = (avDir[i]+1)%6;
              }
          }
      }
        std::cout << "path is: ";
         for(int i=0; i < objectPath.size(); i++)
         std::cout << objectPath.at(i) << ',';

//      if(boundNode.x == 0 && quadrant == 0){
//        quadrant+=1;
//        for(int i = 0; i < 4 ; i++ ){
//            avDir[i] = (avDir[i]+1)%6;
//        }
//      }else if(boundNode.x == boundNode.y  && quadrant == 1){
//         quadrant+=1;
//      }
  }else if(objectShape == ObjectShape::Random_10_1){
      std::vector<int> randomPath = {2,2,2,3,3,2,2,3,3,3,4,4,2,1,1,3,2,1,1,1,3,4,4,2,4,4,4,4,2,2,2,4,5,5,5,5,4,5,5,3,3,3,4,5,5,5,3,3,3,3,5,5,5,5,5,5,0,0,0,0,1,1,5,4,0,1,1,5,0,1,5,1,1,2,1,0,1,2,1,0,2,0,0};
      Node boundNode = Node(10,0);

      for(int i = 0; i < randomPath.size() ; i++){
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(randomPath[i]);
      }
  }else if(objectShape == ObjectShape::Random_10_2){
      std::vector<int> randomPath = {2,3,3,3,3,2,0,0,0,0,2,2,3,3,1,3,3,2,1,2,1,1,3,3,4,4,4,4,2,3,1,1,2,4,4,2,3,2,4,4,2,5,5,4,3,3,5,5,5,5,4,4,3,3,3,3,3,5,5,0,0,4,4,5,5,5,0,0,5,1,1,1,1,5,1,1,1,1,5,0,0,5,5,5,0,0,5,1,1,1,2,1,0};
      Node boundNode = Node(10,0);

      for(int i = 0; i < randomPath.size() ; i++){
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(randomPath[i]);
      }
  }else if(objectShape == ObjectShape::Random_10_3){
      std::vector<int> randomPath = {2,3,3,3,3,1,0,0,1,3,3,1,2,1,2,2,3,3,1,4,4,2,1,1,3,3,4,4,2,1,3,4,4,2,1,3,4,4,4,3,2,2,4,4,4,4,5,5,5,5,5,0,0,5,3,3,4,4,4,4,5,4,0,0,0,5,4,4,0,0,1,1,5,1,1,5,5,5,0,1,1,5,0,2,1,2,2,2,0,2,2,1,0,0,0,0,0};
      Node boundNode = Node(10,0);

      for(int i = 0; i < randomPath.size() ; i++){
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(randomPath[i]);
      }
  }else if(objectShape == ObjectShape::Random_10_4){
      std::vector<int> randomPath = {2,2,2,2,2,2,3,3,2,3,4,4,3,3,4,4,2,4,2,2,2,2,4,4,2,2,3,4,4,5,5,3,2,4,5,5,5,5,3,3,5,4,5,0,0,5,0,0,5,5,5,4,4,0,0,1,1,1,1,1,1,5,0,0,5,0,0,2,2,2,1,2,0,0,5,1,5,1};
      Node boundNode = Node(10,0);

      for(int i = 0; i < randomPath.size() ; i++){
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(randomPath[i]);
      }
  }else if(objectShape == ObjectShape::Random_10_5){
      std::vector<int> randomPath = {2,3,3,3,3,1,1,3,3,1,3,3,2,2,2,2,3,4,4,3,2,1,1,2,3,4,4,2,4,2,4,5,5,4,5,5,4,3,4,2,2,3,4,4,4,0,0,5,0,0,5,4,0,0,0,0,4,5,1,1,1,1,5,0,5,0,5,0,2,0,1,1,5,5,0,1,2,1,};
      Node boundNode = Node(10,0);

      for(int i = 0; i < randomPath.size() ; i++){
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(randomPath[i]);
      }
  }else if(objectShape == ObjectShape::Random_20_1){
        std::vector<int> randomPath = {2,2,3,3,2,0,0,2,3,3,1,1,3,3,2,3,3,2,2,1,3,3,1,1,1,1,2,1,3,3,2,2,2,1,3,4,4,3,4,4,4,4,2,3,3,1,1,2,3,4,4,4,4,3,3,4,4,2,1,1,3,4,4,4,3,3,5,5,4,3,2,2,2,2,3,4,4,3,2,4,5,5,5,5,3,4,4,5,5,3,2,2,3,5,5,3,5,5,4,3,3,3,3,5,0,0,4,5,0,0,0,0,4,5,5,0,0,0,0,5,5,5,0,0,4,5,5,1,1,0,4,4,4,4,0,0,0,4,4,0,0,1,1,1,1,0,5,4,4,5,0,1,1,0,5,4,4,5,1,1,1,1,5,5,5,0,0,0,1,2,0,2,2,2,2,2,1,2,2,1,5,0,2,0,5,5,1,2,1,0,0,0,2,2,0,2,0,1,0,0};
        Node boundNode = Node(10,0);
        for(int i = 0; i < randomPath.size() ; i++){
            insert(new Object(boundNode));
            boundNode = boundNode.nodeInDir(randomPath[i]);
        }
    }else if(objectShape == ObjectShape::Random_20_2){
      std::vector<int> randomPath = {2,3,3,1,2,2,2,0,2,3,3,3,3,3,3,2,1,2,2,1,3,3,1,2,2,1,1,0,0,0,2,2,2,2,3,3,3,4,4,4,4,4,4,4,4,4,4,3,1,1,2,4,4,4,4,2,1,1,3,2,3,2,4,4,4,4,2,3,3,5,5,3,2,2,2,2,3,4,2,2,4,5,5,3,2,2,4,5,5,3,5,5,3,4,2,2,4,5,5,4,3,3,5,5,5,5,4,4,0,0,0,0,5,0,0,4,0,0,4,0,0,4,4,4,3,3,3,3,3,3,4,4,0,0,5,0,0,0,0,5,0,0,4,3,3,3,3,4,5,5,0,0,5,1,1,0,1,1,5,1,1,1,1,5,1,1,0,4,4,0,4,4,0,5,4,4,4,4,4,4,5,0,1,1,1,1,5,4,4,0,0,1,1,5,0,0,5,1,1,1,5,1,5,0,1,5,1,1,1,2,1,2,1,0,1,0,1,2,2,0,5,1,1,2,1,5,1,1,2,2};
      Node boundNode = Node(10,0);
      for(int i = 0; i < randomPath.size() ; i++){
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(randomPath[i]);
      }
  }else if(objectShape == ObjectShape::Random_20_3){
      std::vector<int> randomPath = {2,2,3,3,2,3,3,1,1,3,3,3,3,1,3,3,3,3,2,1,0,0,0,0,1,3,3,1,1,2,2,0,0,2,0,2,3,3,2,1,0,2,2,2,4,4,2,4,4,2,4,4,4,4,4,4,3,4,4,3,1,1,3,2,4,4,3,4,4,3,3,5,5,5,5,4,5,5,4,3,5,3,3,3,3,3,3,4,4,5,0,0,0,0,5,3,3,5,5,4,3,3,4,0,0,4,3,3,4,3,3,5,0,0,4,4,3,3,5,5,4,0,0,4,5,3,5,0,5,1,1,5,5,1,1,0,4,4,0,0,0,0,1,1,1,1,1,1,1,1,0,5,1,1,1,1,1,1,0,5,0,2,0,2,1,0,0,5,1,0,5,1,5,1,2,0,2,2,1,0,0,5,1,0};
      Node boundNode = Node(10,0);
      for(int i = 0; i < randomPath.size() ; i++){
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(randomPath[i]);
      }
  }else if(objectShape == ObjectShape::Random_20_4){
      std::vector<int> randomPath = {2,3,3,3,3,3,3,1,1,2,1,1,1,0,2,3,3,3,3,3,3,1,2,1,3,3,3,3,1,2,3,3,3,2,3,2,3,2,1,1,2,3,3,4,4,3,3,1,1,1,3,4,4,3,1,1,3,3,4,4,3,5,5,4,2,2,4,5,5,3,5,5,4,5,5,5,5,5,5,3,4,3,4,3,2,2,2,2,3,4,4,5,5,5,5,5,0,0,5,5,3,3,5,3,3,5,5,0,0,4,0,0,4,0,0,0,0,4,3,3,3,3,5,3,3,3,3,5,3,3,3,5,5,0,0,4,3,5,5,0,0,4,4,0,0,4,3,5,0,1,1,0,0,0,5,1,1,1,1,0,1,1,1,1,0,0,0,4,4,4,4,0,5,5,1,1,0,4,4,5,0,4,4,5,1,1,1,1,1,2,2,1,1,2,0,2,1,5,1,0,5,1,0,2,2,0,0,2,2,0,1,1,0,1,5,1,5,1,5,1,0};
      Node boundNode = Node(10,0);
      for(int i = 0; i < randomPath.size() ; i++){
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(randomPath[i]);
      }
  }else if(objectShape == ObjectShape::Random_20_5){
      std::vector<int> randomPath = {2,3,3,1,3,3,1,3,3,3,3,3,3,3,3,1,1,1,1,2,1,2,3,3,1,1,0,0,1,0,0,2,3,3,1,3,3,3,3,1,4,4,3,4,4,2,3,2,1,1,1,1,3,2,4,4,4,4,3,4,4,3,1,1,1,1,2,3,4,4,4,4,2,3,4,4,3,5,5,5,5,4,5,5,3,4,5,5,4,5,5,3,3,3,3,3,5,5,3,3,3,3,5,3,3,5,3,3,3,3,4,0,0,5,4,0,0,0,0,0,0,0,0,5,5,0,0,4,0,1,1,1,1,0,5,4,4,5,4,4,0,0,5,4,4,4,4,0,4,4,4,0,0,0,1,1,1,1,1,1,1,1,5,0,0,2,2,0,0,0,5,0,0,0,1,2,1,1,5,0,1,2,2,0,5,1,1,1,2,0,1,2,1};
      Node boundNode = Node(10,0);
      for(int i = 0; i < randomPath.size() ; i++){
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(randomPath[i]);
      }
  }else if(objectShape == ObjectShape::Cross){
      std::vector<int> randomPath = {1,1,1,1,3,3,1,1,1,0,0,1,1,0,0,0,4,4,0,0,4,4,4,3,3,4,4,4,4,3,3,3};
      Node boundNode = Node(0,0);
      for(int i = 0; i < randomPath.size() ; i++){
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(randomPath[i]);
      }
  }else if(objectShape == ObjectShape::Random_Flat_1){
      numberParticles = 500;
      Node boundNode(10, 0);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      for (int i = 0; i <= numberParticles; ++i) {
        insert(new Object(boundNode));
        boundNode = boundNode.nodeInDir(3);
      }
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(4);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(5);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(0);
      for (int i = 0; i < numberParticles; ++i) {
              insert(new Object(boundNode));
              boundNode = boundNode.nodeInDir(0);
      }
  }else if(objectShape == ObjectShape::Random_Flat_2){
      numberParticles = 250-1;
      Node boundNode(10, 0);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      for (int i = 0; i <= numberParticles; ++i) {
        insert(new Object(boundNode));
        boundNode = boundNode.nodeInDir(3);
      }
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(4);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(4);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(5);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(0);
      for (int i = 0; i < numberParticles+1; ++i) {
              insert(new Object(boundNode));
              boundNode = boundNode.nodeInDir(0);
      }


  }else if(objectShape == ObjectShape::Random_Flat_3){
      numberParticles = 166-1;
      Node boundNode(10, 0);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      for (int i = 0; i <= numberParticles; ++i) {
        insert(new Object(boundNode));
        boundNode = boundNode.nodeInDir(3);
      }
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(4);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(4);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(4);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(5);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(0);
      for (int i = 0; i < numberParticles+2; ++i) {
              insert(new Object(boundNode));
              boundNode = boundNode.nodeInDir(0);
      }
  }else if(objectShape == ObjectShape::Random_Flat_4){
      numberParticles = 125;
      Node boundNode(10, 0);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(1);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      for (int i = 0; i <= numberParticles; ++i) {
        insert(new Object(boundNode));
        boundNode = boundNode.nodeInDir(3);
      }
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(5);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(4);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(5);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(4);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(5);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(0);
      for (int i = 0; i < numberParticles; ++i) {
              insert(new Object(boundNode));
              boundNode = boundNode.nodeInDir(0);
      }
  }else if(objectShape == ObjectShape::Random_Flat_5){
      numberParticles = 100-1;
      Node boundNode(10, 0);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(1);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      for (int i = 0; i <= numberParticles; ++i) {
        insert(new Object(boundNode));
        boundNode = boundNode.nodeInDir(3);
      }
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(4);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(5);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(4);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(5);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(4);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(5);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(0);
      for (int i = 0; i < numberParticles+1; ++i) {
              insert(new Object(boundNode));
              boundNode = boundNode.nodeInDir(0);
      }
  }else if(objectShape == ObjectShape::Random_Flat_Generic){
      Node boundNode(10, 0);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(2);
      for (int i = 2; i <= sideLen; ++i) {
        insert(new Object(boundNode));
        boundNode = boundNode.nodeInDir(i%2+1);
//        boundNode = boundNode.nodeInDir(2);
      }
      for (int i = 0; i <= floor (500/sideLen); ++i) {
        insert(new Object(boundNode));
        boundNode = boundNode.nodeInDir(3);
      }
      for (int i = 2; i <= sideLen; ++i) {
        insert(new Object(boundNode));
        boundNode = boundNode.nodeInDir(i%2+4);
//        boundNode = boundNode.nodeInDir(5);
      }
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(5);
      if(sideLen%2 != 0){
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(0);
          for (int i = 0; i <= ((sideLen-3)/2); ++i) {

            insert(new Object(boundNode));
            boundNode = boundNode.nodeInDir(0);
          }
          for (int i = 0; i <= ((sideLen-3)/2); ++i) {
            boundNode = boundNode.nodeInDir(3);
          }
          boundNode = boundNode.nodeInDir(3);
          boundNode = boundNode.nodeInDir(5);
      }else{
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(5);
      }
      for (int i = 0; i < floor (500/sideLen)-3; ++i) {
              insert(new Object(boundNode));
              boundNode = boundNode.nodeInDir(0);
      }

      for (int k = 0; k < (500%sideLen); ++k) {
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(4);
      }

      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(0);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(0);
      insert(new Object(boundNode));
      boundNode = boundNode.nodeInDir(1);

      for (int j = 0; j < (500%sideLen); ++j) {
          insert(new Object(boundNode));
          boundNode = boundNode.nodeInDir(1);
      }
      boundNode = Node(9,-1);
      insert(new Object(boundNode));
  }




  // Let s be the bounding hexagon side length. When the hexagon is created as
  // above, the nodes (x,y) strictly within the hexagon have (i) -s < x < s,
  // (ii) 0 < y < 2s, and (iii) 0 < x+y < 2s. Choose interior nodes at random to
  // place particles, ensuring at most one particle is placed at each node.
  std::set<Node> occupied;


  enum class Shape {
      Compact,
      Line,
      Square,
      Random,
      Random_10_1,
      Random_10_2,
      Random_10_3,
      Random_10_4,
      Random_10_5,
      Random_20_1,
      Random_20_2,
      Random_20_3,
      Random_20_4,
      Random_20_5,
      Random_Flat_1,
      Random_Flat_2,
      Random_Flat_3,
      Random_Flat_4,
      Random_Flat_5,
      Cross,
      Empty,
      Test,
  };
  int shapeInt = static_cast<int>(particleConfigInt);
  Shape shape;
  switch(shapeInt) {
    case 0  :
        shape = Shape::Compact;
        break;
    case 1  :
        shape = Shape::Line;
        break;
      case 2  :
        shape = Shape::Square;
        break;
    case 10  :
        shape = Shape::Random;
        break;
      case 11  :
          shape = Shape::Random_10_1;
          break;
      case 12  :
          shape = Shape::Random_10_2;
          break;
      case 13  :
          shape = Shape::Random_10_3;
          break;
      case 14  :
          shape = Shape::Random_10_4;
          break;
      case 15  :
          shape = Shape::Random_10_5;
          break;
      case 21  :
          shape = Shape::Random_20_1;
          break;
      case 22  :
          shape = Shape::Random_20_2;
          break;
      case 23  :
          shape = Shape::Random_20_3;
          break;
      case 24  :
          shape = Shape::Random_20_4;
          break;
      case 25  :
          shape = Shape::Random_20_5;
          break;
      case 31  :
          shape = Shape::Random_Flat_1;
          break;
      case 32  :
          shape = Shape::Random_Flat_2;
          break;
      case 33  :
          shape = Shape::Random_Flat_3;
          break;
      case 34  :
          shape = Shape::Random_Flat_4;
          break;
      case 35  :
          shape = Shape::Random_Flat_5;
          break;
        case 40  :
          shape = Shape::Cross;
          break;

    case 99  :
      shape = Shape::Test;
      break;
  case 100:
      shape = Shape::Empty;
      break;
    default:
      shape = Shape::Compact;

  }

  int numberParticlesCounter = 0;
  int cond = 0;
  int num_particle_row = 8;
  int num_row = 8;
  if(shape == Shape::Compact){
      Node node(-1, 1);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
      node = Node(-2, 1);
      for(int j = sideLen; j > 0; j--){
         if(j == 1){
             insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
             occupied.insert(node);
         }
//        boundNode = boundNode.nodeInDir(5);
        for (int dir = 5; dir >= 0; --dir) {
          for (int i = 0; i < j-1; i++) {
              insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
              occupied.insert(node);
              if(dir == 0 && i==j-2){
                    node = node.nodeInDir(5);
              }else{
                  node = node.nodeInDir(dir);
              }

            }
          }
        }
    }else if(shape == Shape::Line){
      Node node_leader(-1, 1);
      insert(new FillBoundingBox(node_leader, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node_leader);
      Node node(-2, 1);
//      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Retired));
        for (int y = 0; y < numberParticles; ++y) {
            Node node(-1, 0-y);
            insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
            occupied.insert(node);
        }
    }else if (shape == Shape::Square) {
      Node node_leader(-1, 1);
      insert(new FillBoundingBox(node_leader, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node_leader);
        for (int y = 0; y < num_row; ++y) {
        cond = 8+y;
          for (int x = 0; x < num_particle_row; ++x) {
              if (x == cond) {
                  cond++;
                  continue;
              }else{
                  Node node(0-x-2+y, 0-y+1);
                  insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
                  occupied.insert(node);
                  numberParticlesCounter++;
              }
          }
        }

    }else if(shape == Shape::Test){
      Node node_leader(-1, 1);
      insert(new FillBoundingBox(node_leader, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node_leader);
      Node node(-2, 1);
//      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Retired));
        for (int y = 0; y < numberParticles; ++y) {
            Node node(-1, 0-y);
            insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
            occupied.insert(node);
        }
  }else if(shape == Shape::Random){
      Node node_leader(sideLength-1, 0);
      insert(new FillBoundingBox(node_leader, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node_leader);
      Node node = Node(sideLength+sideLength-1, sideLength-1);
      for(int j = sideLen; j > 0; j--){
         if(j == 1){
             insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
             occupied.insert(node);
         }
//        boundNode = boundNode.nodeInDir(5);
        for (int dir = 5; dir >= 0; --dir) {
          for (int i = 0; i < j-1; i++) {
              insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
              occupied.insert(node);
              if(dir == 0 && i==j-2){
                    node = node.nodeInDir(5);
              }else{
                  node = node.nodeInDir(dir);
              }

            }
          }
        }
  }else if(shape == Shape::Random_10_1){
     Node node(10-1, 0);
     insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
     occupied.insert(node);
       for (int y = 0; y < 139; ++y) {
           node = node.nodeInDir(5);
           insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
           occupied.insert(node);
       }
  }else if(shape == Shape::Random_10_2){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
        for (int y = 0; y < 126; ++y) {
            node = node.nodeInDir(5);
            insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
            occupied.insert(node);
        }
   }else if(shape == Shape::Random_10_3){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
        for (int y = 0; y < 160; ++y) {
            node = node.nodeInDir(5);
            insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
            occupied.insert(node);
        }
   }else if(shape == Shape::Random_10_4){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
        for (int y = 0; y < 138; ++y) {
            node = node.nodeInDir(5);
            insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
            occupied.insert(node);
        }
   }else if(shape == Shape::Random_10_5){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
        for (int y = 0; y < 90; ++y) {
            node = node.nodeInDir(5);
            insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
            occupied.insert(node);
        }
   }else if(shape == Shape::Random_20_1){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
        for (int y = 0; y < 665; ++y) {
            node = node.nodeInDir(5);
            insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
            occupied.insert(node);
        }
   }else if(shape == Shape::Random_20_2){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
        for (int y = 0; y < 676; ++y) {
            node = node.nodeInDir(5);
            insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
            occupied.insert(node);
        }
   }else if(shape == Shape::Random_20_3){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
        for (int y = 0; y < 445; ++y) {
            node = node.nodeInDir(5);
            insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
            occupied.insert(node);
        }
   }else if(shape == Shape::Random_20_4){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
        for (int y = 0; y < 581; ++y) {
            node = node.nodeInDir(5);
            insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
            occupied.insert(node);
        }
   }else if(shape == Shape::Random_20_5){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
        for (int y = 0; y < 485; ++y) {
            node = node.nodeInDir(5);
            insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
            occupied.insert(node);
        }
   }else if(shape == Shape::Random_Flat_1){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
      node = Node(10+12,-12);
      node = node.nodeInDir(4);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
      occupied.insert(node);
      int nodeCount = 1;
      int sideCounter = 0;
      int particleCount = 500;
      while(nodeCount<particleCount){
          for (int i = 0; i < 6; ++i) {
              for(int j = 0; j < sideCounter; j++){

                  if(nodeCount>=particleCount){
                      break;
                  }else{
                      node = node.nodeInDir(i);
                      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
                      nodeCount++;
                      occupied.insert(node);
                  }

              }
          }
          node = node.nodeInDir(4);
          sideCounter++;
      }

   }else if(shape == Shape::Random_Flat_2){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
      node = Node(10+12,-12);
      node = node.nodeInDir(4);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
      occupied.insert(node);
      int nodeCount = 1;
      int sideCounter = 0;
      int particleCount = 500;
      while(nodeCount<particleCount){
          for (int i = 0; i < 6; ++i) {
              for(int j = 0; j < sideCounter; j++){

                  if(nodeCount>=particleCount){
                      break;
                  }else{
                      node = node.nodeInDir(i);
                      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
                      nodeCount++;
                      occupied.insert(node);
                  }

              }
          }
          node = node.nodeInDir(4);
          sideCounter++;
      }
   }else if(shape == Shape::Random_Flat_3){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
      node = Node(10+12,-12);
      node = node.nodeInDir(4);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
      occupied.insert(node);
      int nodeCount = 1;
      int sideCounter = 0;
      int particleCount = 500;
      while(nodeCount<particleCount){
          for (int i = 0; i < 6; ++i) {
              for(int j = 0; j < sideCounter; j++){

                  if(nodeCount>=particleCount){
                      break;
                  }else{
                      node = node.nodeInDir(i);
                      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
                      nodeCount++;
                      occupied.insert(node);
                  }

              }
          }
          node = node.nodeInDir(4);
          sideCounter++;
      }
   }else if(shape == Shape::Random_Flat_4){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
      node = Node(10+12,-12);
      node = node.nodeInDir(4);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
      occupied.insert(node);
      int nodeCount = 1;
      int sideCounter = 0;
      int particleCount = 500;
      while(nodeCount<particleCount){
          for (int i = 0; i < 6; ++i) {
              for(int j = 0; j < sideCounter; j++){

                  if(nodeCount>=particleCount){
                      break;
                  }else{
                      node = node.nodeInDir(i);
                      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
                      nodeCount++;
                      occupied.insert(node);
                  }

              }
          }
          node = node.nodeInDir(4);
          sideCounter++;
      }
   }else if(shape == Shape::Random_Flat_5){
      Node node(10-1, 0);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Leader));
      occupied.insert(node);
      node = Node(10+12,-12);
      node = node.nodeInDir(4);
      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
      occupied.insert(node);
      int nodeCount = 1;
      int sideCounter = 0;
      int particleCount = 500;
      while(nodeCount<particleCount){
          for (int i = 0; i < 6; ++i) {
              for(int j = 0; j < sideCounter; j++){

                  if(nodeCount>=particleCount){
                      break;
                  }else{
                      node = node.nodeInDir(i);
                      insert(new FillBoundingBox(node, -1, 0, *this, FillBoundingBox::State::Inactive));
                      nodeCount++;
                      occupied.insert(node);
                  }

              }
          }
          node = node.nodeInDir(4);
          sideCounter++;
      }
   }else if(shape == Shape::Cross){
      std::vector<int> randomPath = {1,1,1,1,3,3,1,0,0,1,1,0,4,4,0,0,4,3,3,4,4,4,4,4};
      Node boundNode = Node(1,1);
      for(int i = 0; i < randomPath.size() ; i++){
          insert(new FillBoundingBox(boundNode, -1, 0, *this, FillBoundingBox::State::Inactive));
          boundNode = boundNode.nodeInDir(randomPath[i]);
      }
  }else if(shape == Shape::Empty){
      Node boundNode = Node(1,1);
}
}
