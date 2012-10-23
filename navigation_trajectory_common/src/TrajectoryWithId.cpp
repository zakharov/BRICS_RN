
#include "navigation_trajectory_common/TrajectoryWithId.h"
#include "kdl/trajectory.hpp"
#include "kdl/trajectory_composite.hpp"

TrajectoryWithId::TrajectoryWithId() {
    id = "";
    trajectory = new KDL::Trajectory_Composite();
}
    
TrajectoryWithId::TrajectoryWithId(const std::string& id) {
    this->id = id;
    trajectory = new KDL::Trajectory_Composite();
}
    
TrajectoryWithId::TrajectoryWithId(const KDL::Trajectory& trajectory) {
    this->trajectory = trajectory.Clone();
}

TrajectoryWithId::TrajectoryWithId(const KDL::Trajectory& trajectory, const std::string& id) {
    this->id = id;
    this->trajectory = trajectory.Clone();
}
   
const TrajectoryWithId& TrajectoryWithId::operator=(const TrajectoryWithId& orig) {
    id = orig.id;
    delete trajectory;
    trajectory = orig.trajectory;
    
    return *this;
}

TrajectoryWithId::TrajectoryWithId(const TrajectoryWithId& orig) {
    id = orig.id;
    trajectory = orig.getTrajectory().Clone();
}

TrajectoryWithId::~TrajectoryWithId() {
    delete trajectory;
}

    
void TrajectoryWithId::setTrajectory(const KDL::Trajectory& trajectory) {
    delete this->trajectory;
    this->trajectory = trajectory.Clone();
}
 
KDL::Trajectory& TrajectoryWithId::getTrajectory() {
    return *trajectory;
}

const KDL::Trajectory& TrajectoryWithId::getTrajectory() const {
    return *trajectory;
}
