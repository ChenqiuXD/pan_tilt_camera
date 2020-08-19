#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
class Factory : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    sdf::SDF sphereSDF;
     sphereSDF.SetFromString(
        "<?xml version='1.0'?>\
		<sdf version ='1.6'>\
			<model name ='heatmap'>\
				<pose>0 0 0 0 0 0</pose>\
				<link name ='link'>\
					<pose>0 0 0 0 0 0</pose>\
					<gravity> 0 </gravity>\
					<visual name='visual'>\
						<geometry>\
							<mesh>\
								<uri>model://heatmap/meshes/heatmap.dae</uri>\
								<scale>1. 1. 1.</scale>\
							</mesh>\
						</geometry>\
					</visual>\
				</link>\
			</model>\
		</sdf>");
    // Demonstrate using a custom model name.
    const sdf::ElementPtr modelElement = sphereSDF.Root()->GetElement("model");
	sdf::ElementPtr poseElement = modelElement->GetElement("pose");
	poseElement->GetValue()->SetFromString("0 0 0.5 0 0 0");
    _parent->InsertModelSDF(sphereSDF);

	poseElement->GetValue()->SetFromString("0 0 0 1.57 0 0");
	modelElement->GetAttribute("name")->SetFromString("heatmap2");
    _parent->InsertModelSDF(sphereSDF);

  }

};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}
   // _parent->InsertModelSDF(sphereSDF)
