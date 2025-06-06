#pragma once
enum class TaskStage { Idle = 0, ToPickup = 1, ToDelivery = 2 };
struct Task {
  Task()
      : task_id(-1),
        start_time_step(-1),
        pickup_loc(-1),
        delivery_loc(-1),
        stage(TaskStage::Idle) {}
  int task_id;
  int start_time_step;
  int pickup_loc;
  int delivery_loc;
  int assigned_agent_id;
  TaskStage stage;
};
