#include "viewer.h"

Viewer::Viewer()
{
    m_viewer_thread = new thread(&Viewer::ViewerLoop, this);
}

Viewer::~Viewer()
{
    
}

void Viewer::ViewerLoop()
{
    // cout << "viewer thread start" << endl;
    pangolin::CreateWindowAndBind("Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, m_config.view_point_f, m_config.view_point_f, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(m_config.view_point_x, m_config.view_point_y, m_config.view_point_z, 
                                                             m_config.view_center_x, m_config.view_center_y, m_config.view_center_z, 
                                                             m_config.view_up_x, m_config.view_up_y, m_config.view_up_z)
    );
    pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));
	int UI_WIDTH = 175;
	pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
    pangolin::Var<bool> is_show_odo_pose("menu.show_odo_pose", true, true); 
    pangolin::Var<int> grid_scale("menu.Grid Size (m)", 100, 1, 200);
    pangolin::Var<bool> show_grid("menu.Show Grid", false, true);
	pangolin::Var<bool> view_xy_plane("menu.view_xy_plane", false, false);  
	pangolin::Var<bool> view_xz_plane("menu.view_xz_plane", false, false); 
	pangolin::Var<bool> view_yz_plane("menu.view_yz_plane", false, false);  
    while(1)
    {
        if(pangolin::Pushed(view_xy_plane))
		{
			cout << "Pushed button view_xy_plane" << endl;;
			s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0,m_config.view_point_z, 0,0,0, 0,1,0));
		}
        if(pangolin::Pushed(view_xz_plane))
		{
			cout << "Pushed button view_xz_plane" << endl;
			s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,m_config.view_point_z,0, 0,0,0, 0,0,1));
		}
        if(pangolin::Pushed(view_yz_plane))
		{
			cout << "Pushed button view_yz_plane" << endl;
			s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(m_config.view_point_z,0,0, 0,0,0, 0,1,0));
		}
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        pangolin::glDrawAxis(3);
        // Draw grid.
        if (show_grid.Get()) 
        {
            glColor3f(0.3f, 0.3f, 0.3f);
            pangolin::glDraw_z0(grid_scale, 1000);
        }
        // for mutex
        {
            unique_lock<mutex> lock(m_data_mutex);
            if(is_show_odo_pose)
            {
                ShowPoses(mvec_odo_pose);
            }
        }
        pangolin::FinishFrame();
    }
}

void Viewer::UpdateOdoPose(Isometry3d& pos)
{
    unique_lock<mutex> lock(m_data_mutex);
    mvec_odo_pose.push_back(pos);
}

void Viewer::ShowPoses(vector<Isometry3d>& vec_pos)
{
    if(vec_pos.size() == 0) return;
    // draw every point
    for(int i=0; i<vec_pos.size(); i++)
    {
        glPointSize(2.0f);
        glBegin(GL_POINTS);   
        glColor3f(0.0,0.0,1.0); // blue
        Vector3d p = vec_pos[i].translation();
        glVertex3d(p[0], p[1], p[2]);
        glEnd(); 
    }
    // draw coordinate axis for the latest pose
    Vector3d Ow = vec_pos[vec_pos.size()-1].translation();
    Vector3d Xw = vec_pos[vec_pos.size()-1] * (3 * Vector3d(1, 0, 0)); 
    Vector3d Yw = vec_pos[vec_pos.size()-1] * (3 * Vector3d(0, 1, 0));
    Vector3d Zw = vec_pos[vec_pos.size()-1] * (3 * Vector3d(0, 0, 1));
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0); // red
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Xw[0], Xw[1], Xw[2]);
    glColor3f(0.0, 1.0, 0.0); // green
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Yw[0], Yw[1], Yw[2]);
    glColor3f(0.0, 0.0, 1.0); // blue
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Zw[0], Zw[1], Zw[2]);
    glEnd();
}
