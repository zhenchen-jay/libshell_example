#include "CLI11.hpp"
#include <igl/readOBJ.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

struct ViewerArgs
{
	std::string input_folder;       // input folder, inside the folder, the meshes must be named as mesh_0.obj, ..., mesh_99.obj, ....
	std::string input_mesh = "";         // input mesh

	int speed = 30;					// how fast to play the animation
};

ViewerArgs viewerArgs;

Eigen::MatrixXd curPos;
Eigen::MatrixXi curF;
bool isAnimated = false;
int currentFrame = 0;
bool isPaused = false;

bool isShowReferenceCircle = true;

int numPts = 100;
double PI = 3.1415926;
double r = 0.125;
Eigen::MatrixXd circleV(numPts + 1, 3);
Eigen::MatrixXi circleF(numPts, 3);

void repaint(igl::opengl::glfw::Viewer& viewer)
{
	viewer.data().clear();
	viewer.data().set_mesh(curPos, curF);

	if (viewerArgs.input_folder != "")
	{
		viewer.core().is_animating = isAnimated;
		viewer.core().animation_max_fps = viewerArgs.speed > 0 ? viewerArgs.speed : 30;
	}
}

int main(int argc, char* argv[])
{
	// Parse arguments
	CLI::App app("Viewer");
	app.add_option("folder,-f,--folder", viewerArgs.input_folder, "Input folder.");
	app.add_option("mesh,-m,--mesh", viewerArgs.input_mesh, "Input mesh");
	app.add_option("speed,-s,--speed", viewerArgs.speed, "The speed to play the animation (frame per second)");

	std::vector<Eigen::MatrixXd> meshes;
	std::vector<Eigen::MatrixXi> faces;

	igl::opengl::glfw::Viewer viewer;
	viewer.data().set_face_based(false);

	try {
		app.parse(argc, argv);
	}
	catch (const CLI::ParseError& e) {
		return app.exit(e);
	}

	if (viewerArgs.input_mesh != "")
	{
		if (!igl::readOBJ(viewerArgs.input_mesh, curPos, curF))
			return 1;
		else
		{
			isAnimated = false;
			repaint(viewer);
			viewer.launch();
		}

	}
	else if (viewerArgs.input_folder != "")
	{
		isAnimated = true;

		circleV.row(0) << 0, 0, 0;

		for (int i = 0; i < numPts; i++)
		{
			double theta = 1.0 / numPts * i * 2 * PI;
			circleV.row(i + 1) << 0, r * std::sin(theta), r * std::cos(theta);

			if(i != numPts - 1)
				circleF.row(i) << 0, i + 1, i + 2;
			else
				circleF.row(i) << 0, i + 1, 1;
		}

		for (int i = 0; ; i++)
		{
			Eigen::MatrixXd V;
			Eigen::MatrixXi F;

			std::string meshPath = viewerArgs.input_folder + "/mesh_" + std::to_string(i) + ".obj";
			if (!igl::readOBJ(meshPath, V, F))
			{
				std::cout << "successfully load " << i << " meshes." << std::endl;
				break;
			}
			else
			{
				meshes.push_back(V);
				faces.push_back(F);
			}
		}

		// Attach a menu plugin
		igl::opengl::glfw::imgui::ImGuiMenu menu;
		viewer.plugins.push_back(&menu);

		menu.callback_draw_viewer_menu = [&]()
		{
			if (ImGui::CollapsingHeader("Viewing Options", ImGuiTreeNodeFlags_DefaultOpen))
			{
				if (ImGui::Button("Center object", ImVec2(-1, 0)))
				{
					viewer.core().align_camera_center(viewer.data().V, viewer.data().F);
				}
				if (ImGui::Button("Snap canonical view", ImVec2(-1, 0)))
				{
					viewer.snap_to_canonical_quaternion();
				}

				// Select rotation type
				int rotation_type = static_cast<int>(viewer.core().rotation_type);
				//int rotation_type = 0;
				static Eigen::Quaternionf trackball_angle = Eigen::Quaternionf::Identity();
				static bool orthographic = true;
				if (ImGui::Combo("Camera Type", &rotation_type, "Trackball\0Two Axes\02D Mode\0\0"))
				{
					using RT = igl::opengl::ViewerCore::RotationType;
					auto new_type = static_cast<RT>(rotation_type);
					if (new_type != viewer.core().rotation_type)
					{
						if (new_type == RT::ROTATION_TYPE_NO_ROTATION)
						{
							trackball_angle = viewer.core().trackball_angle;
							orthographic = viewer.core().orthographic;
							viewer.core().trackball_angle = Eigen::Quaternionf::Identity();
							viewer.core().orthographic = true;
						}
						else if (viewer.core().rotation_type == RT::ROTATION_TYPE_NO_ROTATION)
						{
							viewer.core().trackball_angle = trackball_angle;
							viewer.core().orthographic = orthographic;
						}
						viewer.core().set_rotation_type(new_type);
					}
				}

				// Orthographic view
				ImGui::Checkbox("Orthographic view", &(viewer.core().orthographic));
			}
			// Draw options
			if (ImGui::CollapsingHeader("Draw Options", ImGuiTreeNodeFlags_DefaultOpen))
			{
				ImGui::ColorEdit4("Background", viewer.core().background_color.data(),
					ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel);
				ImGui::ColorEdit4("Line color", viewer.data().line_color.data(),
					ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel);
				ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
				ImGui::DragFloat("Shininess", &(viewer.data().shininess), 0.05f, 0.0f, 100.0f);
				//ImGui::PopItemWidth();
			}
			// Overlays
			auto make_checkbox = [&](const char* label, unsigned int& option)
			{
				return ImGui::Checkbox(label,
					[&]() { return viewer.core().is_set(option); },
					[&](bool value) { return viewer.core().set(option, value); }
				);
			};

			if (ImGui::CollapsingHeader("Overlays", ImGuiTreeNodeFlags_DefaultOpen))
			{
				make_checkbox("Wireframe", viewer.data().show_lines);
				make_checkbox("Fill", viewer.data().show_faces);
				bool showvertid = viewer.data().show_vertex_labels != 0;
			}
		};

		menu.callback_draw_custom_window = [&]()
		{
			// Define next window position + size
			ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
			ImGui::SetNextWindowSize(ImVec2(0.0, 0.0), ImGuiCond_FirstUseEver);
			ImGui::Begin(
				"Simulation Options", nullptr,
				ImGuiWindowFlags_NoSavedSettings
			);
			if (ImGui::CollapsingHeader("Viewer Parameters", ImGuiTreeNodeFlags_DefaultOpen))
			{
				if (ImGui::Checkbox("Show reference circle", &isShowReferenceCircle))
				{
					repaint(viewer);
				}
				if (ImGui::Button("Pause", ImVec2(-1, 0)))
				{
					viewer.core().is_animating = !isAnimated;
					isPaused = !isPaused;
					std::cout << "current frame: " << currentFrame << std::endl;
					repaint(viewer);
				}
				if (ImGui::Button("Reset", ImVec2(-1, 0)))
				{
					currentFrame = 0;
					repaint(viewer);
				}
				if (ImGui::InputInt("FPS", &viewerArgs.speed))
				{
					ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
					repaint(viewer);
				}
			}
			ImGui::End();
		};

		viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer& viewer)->bool
		{
			curPos = meshes[currentFrame];
			curF = faces[currentFrame];

			if (isShowReferenceCircle)
			{
				int ribbonPts = curPos.rows();
				int circlePts = circleV.rows();

				Eigen::MatrixXd tmpV(ribbonPts + circlePts, 3);
				Eigen::MatrixXi tmpF(curF.rows() + circleF.rows(), 3);

				tmpV.block(0, 0, ribbonPts, 3) = curPos;
				tmpF.block(0, 0, curF.rows(), 3) = curF;

				tmpV.block(ribbonPts, 0, circlePts, 3) = circleV;
				for (int i = 0; i < circleF.rows(); i++)
				{
					tmpF.row(curF.rows() + i) << circleF(i, 0) + ribbonPts, circleF(i, 1) + ribbonPts, circleF(i, 2) + ribbonPts;
				}

				curPos = tmpV;
				curF = tmpF;
			}

			repaint(viewer);
			if(!isPaused)
				currentFrame = (currentFrame + 1) % meshes.size();
			return false;
		};
		viewer.launch();

	}
	else
	{
		std::cout << "please either specify a mesh or a folder." << std::endl;
		return 1;
	}


	return 0;
}
