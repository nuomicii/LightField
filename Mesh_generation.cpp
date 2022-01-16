#define STB_IMAGE_IMPLEMENTATION


#include <string>
#include <fstream>


#include "include/stb_image.h"
#include "include/glm/glm/gtc/type_ptr.hpp"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/IO/write_ply_points.h>


typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Point_2 Point2;
typedef Kernel::Vector_3 Vector;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;
typedef Mesh::Halfedge_index halfedge_descriptor;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Color Color;
typedef Mesh::Property_map <vertex_descriptor, Color> Vertex_map;
typedef std::tuple<Point, Point> PCI; // Point with UVmap
namespace SMP = CGAL::Surface_mesh_parameterization;





int get_pp(std::vector<Point2> *points) {
    //Read the principal points
    for (int i = 0; i <= 120; i++) {
        std::string s = std::to_string(0);
        std::string filename = "D:\\project\\scene1\\depth" + s + ".txt";
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
        std::string sub = line.substr(line.find("principal_point: ") + 17);
        std::istringstream ss(sub);
        std::string s1;
        std::string s2;
        std::getline(ss, s1, ' ');
        std::getline(ss, s2, ',');
        float pp1 = std::stof(s1);
        float pp2 = std::stof(s2);
        Point2 p = CGAL::Point_2<Kernel>(pp1, pp2);
        points->push_back(p);
    }
    return 0;
}


int get_pos(std::string filename, float *D) {
    //get u,v and depth value
    std::ifstream dfile;
    dfile.open(filename);
    std::ostringstream dstr;
    if (!dfile) //checks to see if file opens properly
    {
        std::cerr << "Error: Could not find the requested file.";
    } else {
        dstr << dfile.rdbuf();
    }
    std::string depthstring = dstr.str();
    std::replace(depthstring.begin(), depthstring.end(), ',', ' ');
    std::stringstream ss(depthstring);
    std::string d;
    int i = 0;
    while (ss >> d) {
        int m = i % 480;
        int n = std::floor(i / 480);
        float val = ::atof(d.c_str());
        D[3 * (m + 480 * n)] = (float) n;
        D[3 * (n * 480 + m) + 1] = (float) m;
        D[3 * (n * 480 + m) + 2] = val - 1;
        i++;
    }
    dfile.close(); // Remember to close the file.
    return 0;
}


int get_matrix(float f, float dh, float dv, float pp1, float pp2, int index, glm::mat4 *intrinsic, glm::mat4 *extrinsic) {
    //Original function for performing view transformation by calculating the intrinsic and extrinsic matrices
    //generate intrinsic
    float in[16] = {
            f, 0, 0, 0,
            0, f, 0, 0,
            0, 0, 1, 0,
            pp1, pp2, 0, 1
    };
    glm::mat4 mat;
    memcpy(glm::value_ptr(mat), in, sizeof(in));
    *intrinsic = mat;
    //generate extrinsic
    int y = index % 11;
    int x = std::floor(index / 11);
    float sx = (5 - x) * dh;
    float sy = (-5 + y) * dv;
    float ex[16] = {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            sx, sy, 0, 1
    };
    glm::mat4 mat2;
    memcpy(glm::value_ptr(mat2), ex, sizeof(ex));
    *extrinsic = mat2;
    return 0;
}


int to_xyz(int size, int index, float *pos, std::vector<Point> *points, std::vector<Point2> pp) {
    //convert to xyz coordinate
    for (int i = 0; i < 3 * size; i += 3) {
        Point_2 principal_point = pp[index];
        float pp1 = principal_point[0];
        float pp2 = principal_point[1];
        float x = (pos[i] - pp1 - 1.683) * (pos[i + 2] + 1) * 0.00208333;
        float y = (-pos[i + 1] - pp2 + 1.683) * (pos[i + 2] + 1) * 0.00208333;
        Point p = CGAL::Point_3<Kernel>(x, y, pos[i + 2]);
        points->push_back(p);
    }
    return 0;
}


int store_value(std::vector<Point> points, int w, int h, int index) {
    //generate mesh and store into a ply file
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
            u_map[v] = (float) j / (float) w;;
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


int main() {
    int x = 640;
    int y = 480;
    std::vector<Point2> pp;
    get_pp(&pp);
    for (int i = 0; i <= 120; i++) {
        float *D = (float *) malloc(3 * x * y * sizeof(int));
        std::string st;
        st = std::to_string(i + 12);
        std::string depth = "D:\\project\\scene1\\" + st + ".txt";
        get_pos(depth, D);
        std::vector<Point> points;
        int size = x * y;
        to_xyz(size, i, D, &points, pp);
        std::cout << points.size();
        store_value(points, x, y, i);
        std::cout << "writing" + st << std::endl;
        delete[] D;
    }
    return 0;
}//




