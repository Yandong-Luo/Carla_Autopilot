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
#include <carla/AtomicList.h>

#include "utils/utils.h"
#include "planner/planner.h"

#include<string>
#include<math.h>

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace autopilot::utils;
using namespace autopilot::planner;

#define EXPECT_TRUE(pred) if (!(pred)) { throw std::runtime_error(#pred); }

carla::rpc::EpisodeSettings original_setting;
std::shared_ptr<cc::World> world_ptr_ = nullptr;
std::vector<carla::rpc::ActorId> spawn_actors_id;

// carla::SharedPtr<cc::Actor> ego = nullptr;

/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
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

void signalHandler( int signum )
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    auto all_actors = *world_ptr_->GetActors(spawn_actors_id);
    for(const auto& actor : all_actors){
        actor->Destroy();
    }
    world_ptr_->ApplySettings(original_setting, 10s);
 
  //   // 清理并关闭
  //   // 终止程序  
 
   exit(signum);  
 
}


int main(int argc, const char *argv[]) {
  // 注册信号 SIGINT 和信号处理程序
  signal(SIGINT, signalHandler);  
  try {
    std::string host;
    uint16_t port;
    std::tie(host, port) = ParseArguments(argc, argv);

    std::mt19937_64 rng((std::random_device())());

    auto client = cc::Client(host, port);
    client.SetTimeout(10s);
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
    tm.SetHybridPhysicsRadius(70.0);
    tm.SetGlobalPercentageSpeedDifference(80.0f);
    tm.SetRandomDeviceSeed(1024);
    

    auto settings = carla_world.GetSettings();
    original_setting = settings;
    // sync_mode
    bool sync_mode = true;
    if(sync_mode){
      tm.SetSynchronousMode(true);
      if(settings.synchronous_mode == false){
        settings.synchronous_mode = true;
        double fps = 30.0;
        settings.fixed_delta_seconds = 1.0 / fps;
      }
    }
    carla_world.ApplySettings(settings,3s);

    int npc_number = 5;
    auto map = carla_world.GetMap();
    auto blueprint_library = carla_world.GetBlueprintLibrary();
    auto all_vehicles_bl = blueprint_library->Filter("vehicle");
    auto recSpawnPoint = map->GetRecommendedSpawnPoints();
    auto future_actor = carla::rpc::ActorDescription().uid;
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
      // auto ego = boost::static_pointer_cast<carla::client::ActorBlueprint>(npc_blueprint);
      auto npc_transform = recSpawnPoint[i];

      auto npc_spawn = carla_world.SpawnActor(npc_blueprint, npc_transform);
      auto npc = boost::static_pointer_cast<cc::Vehicle>(npc_spawn);
      npc->SetAutopilot(true, tm.Port());
      spawn_actors_id.emplace_back(npc_spawn->GetId());
      std::cout<<"finish future npc:"<<npc_spawn->GetId()<<"port"<<tm.Port()<<std::endl;
    }

    // excute the command for npc vehicle
    // auto excute_responses = client.ApplyBatchSync(batch_command, true);
    
    // int i = 0;
    // std::cout<<"response size:"<<excute_responses.size()<<std::endl;
    // for(auto response : excute_responses){
    //   std::cout<<"success npc id:"<< response.Get()<<std::endl;
    //   if(!response.HasError()){
    //     i++;
    //     std::cout<<"hi npc id:"<< response.Get()<<"count:"<<i<<std::endl;
    //     // npc_id.emplace_back(response.Get());
    //   }
    // }

    
    // Get the Tesla vehicle blueprint
    auto ego_blueprint = *blueprint_library->Find("vehicle.jeep.wrangler_rubicon");
    // auto blueprint = *ego_blueprint;

    // Randomize the blueprint.
    if (ego_blueprint.ContainsAttribute("color")) {
      auto &attribute = ego_blueprint.GetAttribute("color");
      ego_blueprint.SetAttribute("color", attribute.GetRecommendedValues()[1]);
      // ego_blueprint.SetAttribute(
      //     "color",
      //     RandomChoice(attribute.GetRecommendedValues(), rng));
    }

    if(ego_blueprint.ContainsAttribute("driver_id")){
      auto &attribute = ego_blueprint.GetAttribute("driver_id");
      ego_blueprint.SetAttribute("color", RandomChoice(attribute.GetRecommendedValues(), rng));
    }
    ego_blueprint.SetAttribute("role_name","ego");

    // Find a valid spawn point.

    // auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
    // while(carla_world.TrySpawnActor(ego_blueprint, transform) == nullptr){
    //   std::cout<<"generate new spawn position again"<<std::endl;
    //   transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
    // }
    auto transform = recSpawnPoint[npc_number];
    // Spawn the vehicle.
    auto ego_vehicle = carla_world.SpawnActor(ego_blueprint, transform);
    spawn_actors_id.emplace_back(ego_vehicle->GetId());
    std::cout << "Spawned " << ego_vehicle->GetDisplayId()<<"TypeID "<<ego_vehicle->GetTypeId() << '\n';
    auto ego = boost::static_pointer_cast<cc::Vehicle>(ego_vehicle);
    ego->SetAutopilot(true);
    std::cout<<"ego id"<< ego->GetId()<<std::endl;

    Point3D start_3dPoint(transform.location.x, transform.location.y, transform.location.z);
    // auto end_tramsform = recSpawnPoint[npc_number+1];
    auto end_tramsform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
    Point3D end_3dPoint(end_tramsform.location.x, end_tramsform.location.y, end_tramsform.location.z);

    auto carla_map = carla_world.GetMap();
    Planner m_planner(start_3dPoint, end_3dPoint, carla_map);
    auto route = m_planner.GetRoutePointsByAStar();
    std::cout<<"route size:"<<route.size()<<std::endl;
    PathProfile m_PathProfile;
    // bool pathProfile_result = ComputePathProfile(route, m_PathProfile.headings, m_PathProfile.accumulated_s, m_PathProfile.kappas, m_PathProfile.dkappas);
    bool is_wait_for_tick = false;
    cc::DebugHelper debug(carla_world.GetEpisode());
    for (const auto& loc : route){
      debug.DrawPoint(loc);
    }
    
    while(true){
      // auto all_vehicles = carla_world.GetActors().Filter
      // carla_world.WaitForTick(10min);
      if (!is_wait_for_tick) {
        carla_world.Tick(10.0s);
      } else {
        carla_world.WaitForTick(3min);
      }
      // carla_world.Tick(3s);

      auto ego_transform = ego_vehicle->GetTransform();
      auto spectator = carla_world.GetSpectator();
      auto offset_vector = cg::Location{-10.0 * cos(ego_transform.rotation.yaw * M_PI / 180.0),
                                        -10.0 * sin(ego_transform.rotation.yaw * M_PI / 180.0),
                                        15};
      auto spectator_transform = cg::Transform(ego_transform.location + offset_vector,
                                              cg::Rotation(-45.0,ego_transform.rotation.yaw,0));
      spectator->SetTransform(spectator_transform);

      // boundary box for vehicle
      auto all_vehicles = *carla_world.GetActors(spawn_actors_id);
      for (const auto& vehicle : all_vehicles){
        auto vehicle_transform = vehicle->GetTransform();
        auto bounding_box = vehicle->GetBoundingBox();
        bounding_box.location += vehicle_transform.location;
        debug.DrawBox(bounding_box, bounding_box.rotation, 0.05, cc::DebugHelper::Color{255,0,0}, 0.05, false);
      }

      std::this_thread::sleep_for(0.2s);
    }

    auto all_actors = *world_ptr_->GetActors(spawn_actors_id);
    for(const auto& actor : all_actors){
        actor->Destroy();
    }
    carla_world.ApplySettings(original_setting, 10s);

  } catch (const cc::TimeoutException &e) {
    std::cout << '\n' << e.what() << std::endl;
    return 1;
  } catch (const std::exception &e) {
    std::cout << "\nException: " << e.what() << std::endl;
    return 2;
  }
}
