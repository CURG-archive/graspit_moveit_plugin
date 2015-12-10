#include "include/planning_scene_msg_builder.h"

#include "graspit_source/include/world.h"
#include "graspit_source/include/graspitGUI.h"
#include "graspit_source/include/body.h"
#include "graspit_source/include/triangle.h"
void verticesToMeshMsg(shape_msgs::Mesh &mesh, std::vector<position> *vertices, std::vector<Triangle> *triangles)
{
    int vertice_count = 0;
    for (int i =0 ; i < triangles->size(); i++)
    {
        Triangle t = triangles->at(i);
        shape_msgs::MeshTriangle triangle_msg;

        triangle_msg.vertex_indices[0] = vertice_count;
        geometry_msgs::Point gp0;
        gp0.x = t.v1.x() / 1000;
        gp0.y = t.v1.y() / 1000;
        gp0.z = t.v1.z() / 1000;
        mesh.vertices.push_back(gp0);
        vertice_count ++;

        triangle_msg.vertex_indices[1] = vertice_count;
        geometry_msgs::Point gp1;
        gp1.x = t.v2.x() / 1000;
        gp1.y = t.v2.y() / 1000;
        gp1.z = t.v2.z() / 1000;
        mesh.vertices.push_back(gp1);
        vertice_count ++;

        triangle_msg.vertex_indices[2] = vertice_count;
        geometry_msgs::Point gp2;
        gp2.x = t.v3.x() / 1000;
        gp2.y = t.v3.y() / 1000;
        gp2.z = t.v3.z() / 1000;
        mesh.vertices.push_back(gp2);
        vertice_count ++;

        mesh.triangles.push_back(triangle_msg);
    }
}



PlanningSceneMsgBuilder::PlanningSceneMsgBuilder()
{
}


void PlanningSceneMsgBuilder::uploadPlanningSceneToMoveit()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    int numBodies = graspItGUI->getMainWorld()->getNumGB();

    for(int i =0; i < numBodies; i++)
    {
        Body *b = graspItGUI->getMainWorld()->getGB(i);
        moveit_msgs::CollisionObject collision_obj;
        createCollisionMesh(b, collision_obj);
        collision_objects.push_back(collision_obj);
    }
    planning_scene_interface.addCollisionObjects(collision_objects);
}



bool PlanningSceneMsgBuilder::createCollisionMesh(Body *b, moveit_msgs::CollisionObject &collision_obj)
{
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = QString("base_link").toStdString();
  collision_obj.id = b->getName().toStdString();
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;

  std::vector<position> vertices;
  b->getGeometryVertices(&vertices);

  std::vector<Triangle> triangles;
  b->getGeometryTriangles(&triangles);

  shape_msgs::Mesh mesh;
  verticesToMeshMsg(mesh, &vertices, &triangles);

  collision_obj.meshes.push_back(mesh);

  transf t = b->getTran();
  geometry_msgs::Pose mesh_pose = geometry_msgs::Pose();

  mesh_pose.position.x = t.translation().x() / 1000.0;
  mesh_pose.position.y = t.translation().y() / 1000.0;;
  mesh_pose.position.z = t.translation().z() / 1000.0;;
  mesh_pose.orientation.w = t.rotation().w;
  mesh_pose.orientation.x = t.rotation().x;
  mesh_pose.orientation.y = t.rotation().y;
  mesh_pose.orientation.z = t.rotation().z;
  collision_obj.mesh_poses.push_back(mesh_pose);

}

