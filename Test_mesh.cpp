#include <string>
#include <fstream>
#include "include/stb_image.h"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <CGAL/IO/write_ply_points.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;
typedef Mesh::Halfedge_index halfedge_descriptor;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Color Color;
typedef Mesh::Property_map <vertex_descriptor, Color> Vertex_map;
typedef std::tuple<Point, Point> PCI;


namespace SMP = CGAL::Surface_mesh_parameterization;





int store_test_value(std::vector<Point> points, int w, int h, int index) {
    std::string s = std::to_string(index);
    std::ofstream out2("im" + s + ".ply");
    Mesh m;
    typedef Mesh::Property_map<vertex_descriptor, double> UV_pmap;
    UV_pmap u_map = m.add_property_map<vertex_descriptor, double>("s").first;
    UV_pmap v_map = m.add_property_map<vertex_descriptor, double>("t").first;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            int vi = i * w + j;
            int pos = j * h + i;
            vertex_descriptor v(vi);
            m.add_vertex(points[pos]);
            v_map[v] = (float) i / (float) h;
            u_map[v] = (float) j / (float) w;
        }
    }
    for (int i = 0; i < h - 1; i++) {
        for (int j = 0; j < w - 1; j++) {
            int ui = i * w + j;
            int vi = i * w + j + 1;
            int wi = (i + 1) * w + j + 1;
            int xi = (i + 1) * w + j;
            vertex_descriptor u(ui);
            vertex_descriptor v(vi);
            vertex_descriptor w(wi);
            vertex_descriptor x(xi);
            m.add_face(u, v, w);
            m.add_face(u, w, x);
        }
    }
    CGAL::write_ply(out2, m);
    return 0;
}


int get_test_pos(std::string filename, std::vector<Point> *points) {
    std::ifstream file;
    file.open(filename);
    std::string line;
    std::string v;
    if (!file) //checks to see if file opens properly
    {
        std::cerr << "Error: Could not find the requested file.";
    } else {
        getline(file, line);
        getline(file, line);
    }
    while (getline(file, line)) {
        std::istringstream ss(line);
        std::getline(ss, v, ',');
        std::istringstream mv(v);
        std::getline(mv, v, ':');
        std::getline(mv, v, ':');
        int m = std::stoi(v);
        std::getline(ss, v, ',');
        std::istringstream nv(v);
        std::getline(nv, v, ':');
        std::getline(nv, v, ':');
        int n = std::stoi(v);
        std::getline(ss, v, ',');
        v = v.substr(2, v.length());
        float xi = std::stof(v);
        std::getline(ss, v, ',');
        float yi = std::stof(v);
        std::getline(ss, v, ',');
        v = v.substr(0, v.length() - 2);
        float d = std::stof(v);
        Point p = CGAL::Point_3<Kernel>(xi, yi, d);
        points->push_back(p);
    }
    file.close(); // Remember to close the file.
    return 0;
}


int main() {
    std::string file = "coord/coord0.xyz";
    int w = 640;
    int h = 480;

    for (int i = 0; i <= 120; i++) {
        std::string s = std::to_string(i);
        std::string depth = "D:\\project\\scene1\\depth" + s + ".txt";
        std::vector<Point> points;
        std::string st;
        if (i < 10) {
            st = "00" + s;
        } else if (i < 100) {
            st = "0" + s;
        } else {
            st = s;
        }
        std::string filename = "D:\\project\\scene1\\scene" + st + ".png";
        get_test_pos(depth, &points);
        store_test_value(points, w, h, i);
        std::cout << "writing" + s << std::endl;
    }
    return 0;
}


