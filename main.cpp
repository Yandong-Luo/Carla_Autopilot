/*
 * @Author: Yandong-Luo dongdashu.aa@gmail.com
 * @Date: 2023-01-27 14:26:14
 * @LastEditors: Yandong-Luo dongdashu.aa@gmail.com
 * @LastEditTime: 2024-01-29 23:37:50
 * @FilePath: 
 * @Description: main function
 */

#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <signal.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/ActorList.h>
#include <carla/client/Client.h>
#include <carla/client/DebugHelper.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <carla/rpc/EpisodeSettings.h>
#include <carla/client/World.h>
#include <carla/client/WorldSnapshot.h>
#include <carla/rpc/ActorDescription.h>
#include <carla/rpc/Command.h>
#include "utils/utils.h"

#include<string>
#include<math.h>

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace autopilot::utils;

#define EXPECT_TRUE(pred) if (!(pred)) { throw std::runtime_error(#pred); }

carla::rpc::EpisodeSettings previous_settings;
std::shared_ptr<cc::World> world_ptr_ = nullptr;
carla::SharedPtr<cc::Actor> ego = nullptr;

/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}

void my_handler(int s) {
  printf("Simulation exit\n");
  if (ego != nullptr) {
    ego->Destroy();
  }
  if (world_ptr_ != nullptr) {
    world_ptr_->ApplySettings(previous_settings,3s);
  }
  exit(0);
}

/// Save a semantic segmentation image to disk converting to CityScapes palette.
/*
static void SaveSemSegImageToDisk(const csd::Image &image) {
  using namespace carla::image;

  char buffer[9u];
  std::snprintf(buffer, sizeof(buffer), "%08zu", image.GetFrame());
  auto filename = "_images/"s + buffer + ".png";

  auto view = ImageView::MakeColorConvertedView(
      ImageView::MakeView(image),
      ColorConverter::CityScapesPalette());
  ImageIO::WriteView(filename, view);
}
*/

static auto ParseArguments(int argc, const char *argv[]) {
  EXPECT_TRUE((argc == 1u) || (argc == 3u));
  using ResultType = std::tuple<std::string, uint16_t>;
  return argc == 3u ?
      ResultType{argv[1u], std::stoi(argv[2u])} :
      ResultType{"localhost", 2000u};
}

// void initialize_scenario(cc::Client client, std::string town_name, std::string vehicle_type, std::mt19937_64 rng){
//   // Load the town 04.
//   std::cout << "Loading world: " << town_name << std::endl;
//   auto carla_world = client.LoadWorld(town_name);

//   // Get the Tesla vehicle blueprint
//   auto ego_blueprint = carla_world.GetBlueprintLibrary()->Find(vehicle_type);
//   auto blueprint = *ego_blueprint;

//   // Randomize the blueprint.
//   if (blueprint.ContainsAttribute("color")) {
//     auto &attribute = blueprint.GetAttribute("color");
//     blueprint.SetAttribute("color", "black");
//     // blueprint->SetAttribute(
//     //     "color",
//     //     RandomChoice(attribute.GetRecommendedValues(), rng));
//   }

//   // Find a valid spawn point.
//   auto map = carla_world.GetMap();
//   auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);

//   // Spawn the vehicle.
//   auto actor = carla_world.SpawnActor(blueprint, transform);
//   std::cout << "Spawned " << actor->GetDisplayId() << '\n';
//   // auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);

//   // add camera
// }

// execution loop for synchronous mode
void synchronous_mode_update_thread(){
  while(true){

  }
}

// void update_clock(cc:WorldSnapshot world_snapshot){

// }

int main(int argc, const char *argv[]) {
  // signal(SIGINT, my_handler);
  try {
    std::string host;
    uint16_t port;
    std::tie(host, port) = ParseArguments(argc, argv);

    std::mt19937_64 rng((std::random_device())());

    auto client = cc::Client(host, port);
    client.SetTimeout(20s);
    std::cout << "Client API version : " << client.GetClientVersion() << '\n';
    std::cout << "Server API version : " << client.GetServerVersion() << '\n';

    // vehicle.jeep.wrangler_rubicon
    // initialize_scenario(client, "Town04", "vehicle.tesla.model3", rng);

    // Load the town 04.
    auto carla_world = client.LoadWorld("Town01");
    world_ptr_ = std::shared_ptr<cc::World>(&carla_world);

    // traffic managerment
    // 管辖车的整体行为
    // traffic manager里的每一辆车都要和前车保持至少3m的距离来保持安全
    carla::traffic_manager::TrafficManager tm(carla_world.GetEpisode());
    tm.SetGlobalDistanceToLeadingVehicle(3.0f);
    tm.SetHybridPhysicsMode(true);
    tm.SetGlobalPercentageSpeedDifference(80.0f);
    tm.SetSynchronousMode(true);

    int npc_number = 50;
    auto map = carla_world.GetMap();
    auto blueprint_library = carla_world.GetBlueprintLibrary();
    auto all_vehicles_bl = blueprint_library->Filter("vehicle");
    std::vector<carla::rpc::Command> batch_command;
    for(int i = 0; i < npc_number; ++i){
      auto npc_blueprint = RandomChoice(*all_vehicles_bl, rng);
      // Randomize the npc_blueprint color.
      if (npc_blueprint.ContainsAttribute("color")) {
        auto &attribute = npc_blueprint.GetAttribute("color");
        npc_blueprint.SetAttribute(
            "color",
            RandomChoice(attribute.GetRecommendedValues(), rng));
      }
      // Randomize the npc id.
      if (npc_blueprint.ContainsAttribute("driver_id")) {
        auto &attribute = npc_blueprint.GetAttribute("driver_id");
        npc_blueprint.SetAttribute(
            "driver_id",
            RandomChoice(attribute.GetRecommendedValues(), rng));
      }
      npc_blueprint.SetAttribute("role_name","autopilot");
      auto npc_transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
      while(carla_world.TrySpawnActor(npc_blueprint, npc_transform) == nullptr){
        npc_transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
      }
      auto npc_description = npc_blueprint.MakeActorDescription();
      carla::rpc::Command::SpawnActor npc_spawn(npc_description, npc_transform);
      carla::rpc::Command::SetAutopilot auto_npc(npc_description.uid, true, tm.Port());

      batch_command.emplace_back(npc_spawn);
      batch_command.emplace_back(auto_npc);
      std::cout<<"finish npc:"<<i<<std::endl;
    }

    // excute the command for npc vehicle
    auto excute_response = client.ApplyBatchSync(batch_command, true);

    auto settings = carla_world.GetSettings();
    previous_settings = settings;
    // sync_mode
    bool sync_mode = true;
    if(sync_mode){
      settings.synchronous_mode = true;
      double fps = 30.0;
      settings.fixed_delta_seconds = 1.0 / fps;
    }
    
    carla_world.ApplySettings(settings,3s);

    // Get the Tesla vehicle blueprint
    auto ego_blueprint = *blueprint_library->Find("vehicle.jeep.wrangler_rubicon");
    // auto blueprint = *ego_blueprint;

    // Randomize the blueprint.
    if (ego_blueprint.ContainsAttribute("color")) {
      auto &attribute = ego_blueprint.GetAttribute("color");
      // ego_blueprint.SetAttribute("color", "0,255,0");
      ego_blueprint.SetAttribute(
          "color",
          RandomChoice(attribute.GetRecommendedValues(), rng));
    }

    // Find a valid spawn point.
    auto point = map->GetRecommendedSpawnPoints();
    std::cout<<"spawn point size:"<<point.size()<<std::endl;
    auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
    while(carla_world.TrySpawnActor(ego_blueprint, transform) == nullptr){
      std::cout<<"generate new spawn position again"<<std::endl;
      transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
    }
    // Spawn the vehicle.
    auto ego_vehicle = carla_world.SpawnActor(ego_blueprint, transform);
    std::cout << "Spawned " << ego_vehicle->GetDisplayId() << '\n';
    auto ego = boost::static_pointer_cast<cc::Vehicle>(ego_vehicle);

    // }
    bool is_wait_for_tick = false;
    cc::DebugHelper debug(carla_world.GetEpisode());
    while(true){
      // auto all_vehicles = carla_world.GetActors().Filter
      // carla_world.WaitForTick(10min);
      if (!is_wait_for_tick) {
        carla_world.Tick(10.0s);
      } else {
        carla_world.WaitForTick(10min);
      }

      auto ego_transform = ego->GetTransform();
      auto spectator = carla_world.GetSpectator();
      auto offset_vector = cg::Location{-10.0 * cos(ego_transform.rotation.yaw * M_PI / 180.0),
                                        -10.0 * sin(ego_transform.rotation.yaw * M_PI / 180.0),
                                        15};
      auto spectator_transform = cg::Transform(ego_transform.location + offset_vector,
                                              cg::Rotation(-45.0,ego_transform.rotation.yaw,0));
      spectator->SetTransform(spectator_transform);

      // boundary box for vehicle
      auto all_vehicles = *carla_world.GetActors();
      std::cout<<"the number of box:"<<all_vehicles.size()<<std::endl;
      for (const auto& vehicle : all_vehicles){
        auto vehicle_transform = vehicle->GetTransform();
        auto bounding_box = vehicle->GetBoundingBox();
        bounding_box.location += vehicle_transform.location;
        debug.DrawBox(bounding_box, bounding_box.rotation);
      }
    }

    // Apply control to vehicle.
    // cc::Vehicle::Control control;
    // control.throttle = 1.0f;
    // vehicle->ApplyControl(control);

    // // Move spectator so we can see the vehicle from the simulator window.
    // auto spectator = world.GetSpectator();
    // transform.location += 32.0f * transform.GetForwardVector();
    // transform.location.z += 2.0f;
    // transform.rotation.yaw += 180.0f;
    // transform.rotation.pitch = -15.0f;
    // spectator->SetTransform(transform);

/*
    // Find a camera blueprint.
    auto camera_bp = blueprint_library->Find("sensor.camera.semantic_segmentation");
    EXPECT_TRUE(camera_bp != nullptr);

    // Spawn a camera attached to the vehicle.
    auto camera_transform = cg::Transform{
        cg::Location{-5.5f, 0.0f, 2.8f},   // x, y, z.
        cg::Rotation{-15.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
    auto cam_actor = world.SpawnActor(*camera_bp, camera_transform, actor.get());
    auto camera = boost::static_pointer_cast<cc::Sensor>(cam_actor);

    // Register a callback to save images to disk.
    camera->Listen([](auto data) {
        auto image = boost::static_pointer_cast<csd::Image>(data);
        EXPECT_TRUE(image != nullptr);
        SaveSemSegImageToDisk(*image);
    });

    std::this_thread::sleep_for(10s);

    // Remove actors from the simulation.
    camera->Destroy();
*/
    ego->Destroy();
    std::cout << "Actors destroyed." << std::endl;

  } catch (const cc::TimeoutException &e) {
    std::cout << '\n' << e.what() << std::endl;
    return 1;
  } catch (const std::exception &e) {
    std::cout << "\nException: " << e.what() << std::endl;
    return 2;
  }
}
