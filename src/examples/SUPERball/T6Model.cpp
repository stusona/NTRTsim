/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 *
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file T6Model.cpp
 * @brief Contains the implementation of class T6Model.
 * $Id$
 */

// This module
#include "T6Model.h"

// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"

// The Bullet Physics library
#include "LinearMath/btVector3.h"

// The C++ Standard Library
#include <stdexcept>

namespace
{
    // see tgBasicActuator and tgRod for a descripton of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    // Note: This current model of the SUPERball rod is 1.5m long by 3 cm radius,
    // which is 0.00424 m^3.
    // For SUPERball v1.5, mass = 3.5kg per strut, which comes out to
    // 0.825 kg / (decimeter^3).

    // similarly, frictional parameters are for the tgRod objects.
    const struct Config
    {
        bool   hist;
        double density;
        double radius;
        double stiffness;
        double damping;
        double rod_length;
        double rod_space;
        double friction;
        double rollFriction;
        double restitution;
        double pretension;
        double maxTens;
        double targetVelocity;
        double payloadDensity;
        double payloadLength;
        double payloadRadius;
        double payloadStiffness;
        double payloadDamping;
        double payloadPretension;
    } c =
   {
     0,	       // History logging (boolean)
     0.881,    // rod density (kg / length^3)
     0.0476,   // rod radius (length)
     85.0,     // outer spring stiffness (kg / sec^2)
     200.0,    // outer spring damping (kg / sec)
     5,        // rod length (length)
     2.5/2,    // rod spacing (length)
     0.99,     // rod friction (unitless)
     0.1,      // rod rolling friction (unitless)
     0.8,      // outer spring restitution (unitless)
     50.0,     // pretension of outer springs
     100000,   // maxTens
     10000,    // targetVelocity
     0.5/3,    // payloadDensity
     0.8,      // payloadLength
     0.4,      // payloadRadius
     880.0,    // payloadStiffness
     200.0,    // payloadDamping
     140.0,    // payloadPretension
  };
} // namespace

T6Model::T6Model() : tgModel()
{
}

T6Model::~T6Model()
{
}

void T6Model::addNodes(tgStructure& s)
{
    // half the length of the rods
    const double half_length = c.rod_length / 2;

    // create nodes at each end of each of the 6 rods
    s.addNode(-c.rod_space,  -half_length, 0);            // 0
    s.addNode(-c.rod_space,   half_length, 0);            // 1
    s.addNode( c.rod_space,  -half_length, 0);            // 2
    s.addNode( c.rod_space,   half_length, 0);            // 3
    s.addNode(0,           -c.rod_space,   -half_length); // 4
    s.addNode(0,           -c.rod_space,    half_length); // 5
    s.addNode(0,            c.rod_space,   -half_length); // 6
    s.addNode(0,            c.rod_space,    half_length); // 7
    s.addNode(-half_length, 0,            c.rod_space);   // 8
    s.addNode( half_length, 0,            c.rod_space);   // 9
    s.addNode(-half_length, 0,           -c.rod_space);   // 10
    s.addNode( half_length, 0,           -c.rod_space);   // 11

    // create nodes at the 6 sides of the payload
    // Payload is made out of 3 cylinders aligned with the 3 axes
    s.addNode(-c.payloadLength/2, 0, 0);                  // 12
    s.addNode( c.payloadLength/2, 0, 0);                  // 13
    s.addNode(0, -c.payloadLength/2, 0);                  // 14
    s.addNode(0,  c.payloadLength/2, 0);                  // 15
    s.addNode(0, 0, -c.payloadLength/2);                  // 16
    s.addNode(0, 0,  c.payloadLength/2);                  // 17
    //*/
}

void T6Model::addRods(tgStructure& s)
{
    // create the 6 rods
    s.addPair( 0,  1, "rod");
    s.addPair( 2,  3, "rod");
    s.addPair( 4,  5, "rod");
    s.addPair( 6,  7, "rod");
    s.addPair( 8,  9, "rod");
    s.addPair(10, 11, "rod");

    // create the 3 payloads
    s.addPair(12, 13, "payload");
    s.addPair(14, 15, "payload");
    s.addPair(16, 17, "payload");

    // create the 'rods' that hold the 6 payload nodes together
    s.addPair(12, 14, "massless");
    s.addPair(12, 15, "massless");
    s.addPair(12, 16, "massless");
    s.addPair(12, 17, "massless");

    s.addPair(13, 14, "massless");
    s.addPair(13, 15, "massless");
    s.addPair(13, 16, "massless");
    s.addPair(13, 17, "massless");

    s.addPair(14, 16, "massless");
    s.addPair(14, 17, "massless");
    s.addPair(15, 16, "massless");
    s.addPair(15, 17, "massless");
}

void T6Model::addActuators(tgStructure& s)
{
    // create the 24 "muscles" between rods
    s.addPair(0, 4,  "muscle");
    s.addPair(0, 5,  "muscle");
    s.addPair(0, 8,  "muscle");
    s.addPair(0, 10, "muscle");

    s.addPair(1, 6,  "muscle");
    s.addPair(1, 7,  "muscle");
    s.addPair(1, 8,  "muscle");
    s.addPair(1, 10, "muscle");

    s.addPair(2, 4,  "muscle");
    s.addPair(2, 5,  "muscle");
    s.addPair(2, 9,  "muscle");
    s.addPair(2, 11, "muscle");

    s.addPair(3, 7,  "muscle");
    s.addPair(3, 6,  "muscle");
    s.addPair(3, 9,  "muscle");
    s.addPair(3, 11, "muscle");

    s.addPair(4, 10, "muscle");
    s.addPair(4, 11, "muscle");

    s.addPair(5, 8,  "muscle");
    s.addPair(5, 9,  "muscle");

    s.addPair(6, 10, "muscle");
    s.addPair(6, 11, "muscle");

    s.addPair(7, 8,  "muscle");
    s.addPair(7, 9,  "muscle");

    // create the 12 cables between the payload and rod ends
    s.addPair( 8, 12, "cable");
    s.addPair(10, 12, "cable");
    s.addPair( 9, 13, "cable");
    s.addPair(11, 13, "cable");

    s.addPair( 0, 14, "cable");
    s.addPair( 2, 14, "cable");
    s.addPair( 1, 15, "cable");
    s.addPair( 3, 15, "cable");

    s.addPair( 4, 16, "cable");
    s.addPair( 6, 16, "cable");
    s.addPair( 5, 17, "cable");
    s.addPair( 7, 17, "cable");
}

void T6Model::setup(tgWorld& world)
{
    // Configure rods
    const tgRod::Config rodConfig(c.radius, c.density, c.friction,
				c.rollFriction, c.restitution);

    // configure payload elements
    const tgRod::Config payloadConfig(c.payloadRadius, c.payloadDensity,
        c.friction, c.rollFriction, c.restitution);

    // configure massless elements that hold payloads together
    const tgRod::Config masslessConfig(0.01, 0.0001, 1, 0.0001, 0);


    // @todo acceleration constraint was removed on 12/10/14
    // Replace with tgKinematicActuator as appropreate
    tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension,
        c.hist, c.maxTens, c.targetVelocity);
    tgBasicActuator::Config cableConfig(c.payloadStiffness, c.payloadDamping,
        c.payloadPretension, c.hist, c.maxTens, c.targetVelocity);

    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
    addActuators(s);

    // move model up to initial position
    s.move(btVector3(0, 20, 0)); // Y direction: decimeters above ground

    // Add a rotation. This is needed if the ground slopes too much,
    // otherwise  glitches put a rod below the ground.
    btVector3 rotationPoint = btVector3(0, 0, 0);  // origin
    btVector3 rotationAxis  = btVector3(1, 0, 0);  // x-axis
    double    rotationAngle = M_PI/4;
    s.addRotation(rotationPoint, rotationAxis, rotationAngle);

    // Create the build spec that uses tags to turn the structure
    // into a real model
    tgBuildSpec spec;

    // rods
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));

    // payload
    spec.addBuilder("payload", new tgRodInfo(payloadConfig));
    spec.addBuilder("cable", new tgBasicActuatorInfo(cableConfig));
    spec.addBuilder("massless", new tgRodInfo(masslessConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control.
    allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void T6Model::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children

        // std::cout << allActuators.size() << std::endl;
        for (int k = 0; k < allActuators.size()-1; k++) {
          //std::cout << k << ": " << actuators[k]->getTension() << ", ";
          //std::cout << allActuators[k] -> getTension() << ", ";
        }

        // print actuator tensions to terminal
        //std::cout << allActuators[allActuators.size()-1] -> getTension() << std::endl;
    }
}

void T6Model::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& T6Model::getAllActuators() const
{
    return allActuators;
}

void T6Model::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
