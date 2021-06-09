#ifndef WHOLE_BODY_PLANNING_SERVER_EXECUTION_MANAGER
#define WHOLE_BODY_PLANNING_SERVER_EXECUTION_MANAGER

#include <moveit/move_group/move_group_context.h>

namespace whole_body_planning_server {

class ExecutionManager {
public:
private:
  move_group::MoveGroupContextPtr move_group_context_ptr_;
};

}

#endif
