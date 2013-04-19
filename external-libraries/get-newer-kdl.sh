#! /bin/bash

if [ ! -e get-newer-kdl.sh ] ; then
    echo "Must be run in the BRICS_RN external-libraries folder"
    exit 1
fi

git clone http://git.mech.kuleuven.be/robotics/orocos_kinematics_dynamics.git
cd orocos_kinematics_dynamics
rosmake orocos_kinematics_dynamics

echo "Done."

