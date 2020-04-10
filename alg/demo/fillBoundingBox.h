#ifndef AMOEBOTSIM_ALG_DEMO_FILLBOUNDINGBOX_H_
#define AMOEBOTSIM_ALG_DEMO_FILLBOUNDINGBOX_H_
/* Copyright (C) 2020 Joshua J. Daymude, Robert Gmyr, and Kristian Hinnenthal.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

// Defines the particle system and composing particles for the Disco code
// tutorial, a first algorithm for new developers to AmoebotSim. Disco
// demonstrates the basics of algorithm architecture, instantiating a particle
// system, moving particles, and changing particles' states. The pseudocode is
// available in the docs:
// [https://amoebotsim.rtfd.io/en/latest/tutorials/tutorials.html#discodemo-your-first-algorithm].
//
// Run on the simulator command line using discodemo(# particles, counter max).


#include <QString>

#include "core/amoebotparticle.h"
#include "core/amoebotsystem.h"

class FillBoundingBox : public AmoebotParticle {
 public:
  enum class State {
    Leader,
    Inactive,
    Retired,
    Leaf,
    Follower,
    Coater,
    Branch,
    Root,
    Sroot,
  };

  // Constructs a new particle with a node position for its head, a global
  // compass direction from its head to its tail (-1 if contracted), an offset
  // for its local compass, a system that it belongs to, and a maximum value for
  // its counter.
  FillBoundingBox(const Node head, const int globalTailDir,
                    const int orientation, AmoebotSystem& system,
                    State state);

  // Executes one particle activation.
  void activate() override;

  // Functions for altering the particle's color. headMarkColor() (resp.,
  // tailMarkColor()) returns the color to be used for the ring drawn around the
  // particle's head (resp., tail) node. In this demo, the tail color simply
  // matches the head color.
  int headMarkColor() const override;
  int tailMarkColor() const override;
  int headMarkDir() const override;

  // Returns the string to be displayed when this particle is inspected; used to
  // snapshot the current values of this particle's memory at runtime.
  QString inspectionText() const override;

  // labelOfFirstNbrInState returns the label of the first port incident to a
  // neighboring particle in any of the specified states, starting at the
  // (optionally) specified label and continuing counterclockwise.
  int labelOfFirstNbrInState(std::initializer_list<State> states,
                             int startLabel = 0) const;

  // Checks whether this particle has a neighbor in any of the given states.
  bool hasNbrInState(std::initializer_list<State> states) const;

  // Get move direction of local nbr
  int getPointDir(FillBoundingBox& nbr) const;

  // Gets a reference to the neighboring particle incident to the specified port
  // label. Crashes if no such particle exists at this label; consider using
  // hasNbrAtLabel() first if unsure.
  FillBoundingBox& nbrAtLabel(int label) const;

  //Gets the labels of the neighboring particles incident to the specified port
  // label point to the particle calling the fucntion.
  std::set<int> findFollowerChildren() const;

  // Check if particle can pull label
  bool canPullParticle(int label) const;

  // Update follower parameters, (called after pull particle)
  void updateFollower(int label, int moveDir) const;

  // check if particle is a Leaf
  bool ifLeaf() const;

  int checkLabel(int label) const;

  int getFollowerLabel() const;

  int getExpandLabel() const;

  int getPullLabel() const;

protected:
  // Returns a random State.
  State getRandColor() const;

  // Member variables.
  State _state;
  bool _ready;
  int _moveDir;
  int _moveDirExp;

 private:
  friend class FillBoundingBoxSystem;
};

class FillBoundingBoxSystem : public AmoebotSystem {
 public:
  // Constructs a system of the specified number of DiscoDemoParticles enclosed
  // by a hexagonal ring of objects.
  FillBoundingBoxSystem(unsigned int numParticles = 30);
};

#endif // 2DDFILL_H
