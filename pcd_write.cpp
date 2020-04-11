#include <iostream>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/io/vtk_lib_io.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;

static const string METHOD_OF_POISSON = "p";
static const string METHOD_OF_GRID_PROJECTION = "gp";

//default parameters
bool default_use_kd_tree = true;
//default poisson parameters
int default_depth = 8;
int default_solver_divide = 8;
int default_iso_divide = 8;
float default_point_weight = 4.0f;
//default grid projection parameters
float default_resolution = 0.005;
int default_padding_size = 3;
int default_nearest_neighbor_no = 100;
int default_max_binary_search_level = 10;
//default method is poisson
string default_method = METHOD_OF_POISSON;

void printHelp(int, char **argv) {
    print_error("Syntax is: %s input.pcd output.vtk <options>\n", argv[0]);
    print_info("  where options are:\n");
    print_info("                     -depth X          = set the maximum depth of the tree that will be used for surface reconstruction (default: ");
    print_value("%d", default_depth);
    print_info(")\n");
    print_info("                     -solver_divide X  = set the the depth at which a block Gauss-Seidel solver is used to solve the Laplacian equation (default: ");
    print_value("%d", default_solver_divide);
    print_info(")\n");
    print_info("                     -iso_divide X     = Set the depth at which a block iso-surface extractor should be used to extract the iso-surface (default: ");
    print_value("%d", default_iso_divide);
    print_info(")\n");
    print_info("                     -point_weight X   = Specifies the importance that interpolation of the point samples is given in the formulation of the screened Poisson equation. The results of the original (unscreened) Poisson Reconstruction can be obtained by setting this value to 0. (default: ");
    print_value("%f", default_point_weight);
    print_info(")\n");
}

bool readPLY(const string &filename, PointCloud<PointNormal> &cloud) {
    if (loadPLYFile(filename, cloud) < 0)
        return (false);
    return (true);
}

void computePoisson(const PointCloud<PointNormal>::ConstPtr &input, PolygonMesh &output,
        int depth, int solver_divide, int iso_divide, float point_weight, bool use_kd_tree) {
    print_info("Using parameters: depth %d, solver_divide %d, iso_divide %d, point_weight %d, use_kd_tree %d\n",
            depth, solver_divide, iso_divide, point_weight, use_kd_tree);
    Poisson<PointNormal> poisson;
    // Set parameters
    if(use_kd_tree) {
    // Create search tree*
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(input);
        poisson.setSearchMethod(tree2);
    }
    poisson.setDepth(depth);
    poisson.setSolverDivide(solver_divide);
    poisson.setIsoDivide(iso_divide);
    poisson.setPointWeight(point_weight);
    poisson.setInputCloud(input);

    TicToc tt;
    print_highlight("Computing ...");
    tt.tic();
    poisson.reconstruct(output);
    print_info("[Done, ");
    print_value("%g", tt.toc());
    print_info(" ms]\n");
}

void computeGridProjection(const PointCloud<PointNormal>::ConstPtr &input, PolygonMesh &output, float resolution,
        int paddingSize, int nearestNeighborNo, int maxBinarySearchLevel, bool use_kd_tree) {

    print_info("Using parameters: resolution %.6f, padding size %d, nearest neighbor number %d, max binary search level %d, use_kd_tree %d\n",
            resolution, paddingSize, nearestNeighborNo, maxBinarySearchLevel, use_kd_tree);
    // Set parameters
    GridProjection<PointNormal> grid;
    if(use_kd_tree) {
    // Create search tree*
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(input);
        grid.setSearchMethod(tree2);
    }
    grid.setResolution(resolution);
    grid.setPaddingSize(paddingSize);
    grid.setNearestNeighborNum(nearestNeighborNo);
    grid.setMaxBinarySearchLevel(maxBinarySearchLevel);
    grid.setInputCloud(input);

    TicToc tt;
    tt.tic();
    print_highlight("Computing ...");
    grid.reconstruct(output);
    print_info("[Done, ");
    print_value("%g", tt.toc());
    print_info(" ms]\n");

}

void saveSurface(const string &filename, const PolygonMesh &output, bool isObj) {
    TicToc tt;
    tt.tic();

    print_highlight("Saving ");
    print_value("%s ", filename.c_str());
    
    if (isObj)
        saveOBJFile(filename, output); 
    else 
        savePolygonFileSTL(filename, output);
    
    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms]\n");
}

bool readXYZwithNormals(const string &filename, PointCloud<PointNormal> &out_cloud) {
    //XYZcloud
    PointCloud<PointXYZ> xyz_cloud;
    PointCloud<Normal> normal_cloud;
    ifstream fs;
    fs.open(filename.c_str(), ios::binary);
    if (!fs.is_open() || fs.fail()) {
        PCL_ERROR("Could not open file '%s'! Error : %s\n", filename.c_str(), strerror(errno));
        fs.close();
        return (false);
    }

    string line;
    vector<string> st;

    while (!fs.eof()) {
        getline(fs, line);
        // Ignore empty lines
        if (line.empty())
            continue;

        // Tokenize the line
        boost::trim(line);
        boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

        if (st.size() != 6)
            continue;

        xyz_cloud.push_back(PointXYZ(float (atof(st[0].c_str())), float (atof(st[1].c_str())), float (atof(st[2].c_str()))));
        normal_cloud.push_back(Normal(float (atof(st[3].c_str())), float (atof(st[4].c_str())), float (atof(st[5].c_str()))));
    }
    fs.close();

    xyz_cloud.width = uint32_t(xyz_cloud.size());
    xyz_cloud.height = 1;
    xyz_cloud.is_dense = true;

    normal_cloud.width = uint32_t(normal_cloud.size());
    normal_cloud.height = 1;
    normal_cloud.is_dense = true;

    concatenateFields(xyz_cloud, normal_cloud, out_cloud);

    //TEST
//    cerr << "Cloud out: " << out_cloud.points.size() << endl;
//    for (size_t i = 0; i < out_cloud.points.size(); ++i)
//        cerr << "    " <<
//            out_cloud.points[i].x << " " << out_cloud.points[i].y << " " << out_cloud.points[i].z << " " <<
//            out_cloud.points[i].normal[0] << " " << out_cloud.points[i].normal[1] << " " << out_cloud.points[i].normal[2] << endl;
    return (true);
}

void createCloudWithNormals(const string filename, PointCloud<PointNormal> &out_cloud, bool isPly) {
    TicToc tt;
    print_highlight("Loading ");
    print_value("%s ", filename.c_str());
    if (!isPly)
        readXYZwithNormals(filename, out_cloud);
    else
        readPLY(filename, out_cloud);
    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms : ");
    print_value("%d", out_cloud.width * out_cloud.height);
    print_info(" points]\n");
    print_info("Available dimensions: ");
    print_value("%s\n", pcl::getFieldsList(out_cloud).c_str());
}

void parsePoisson(int argc, char** argv, int &depth, int &solver_divide, int &iso_divide, float &point_weight, bool &use_kd_tree) {
    parse_argument(argc, argv, "-depth", depth);
    print_info("Using a depth of: ");
    print_value("%d\n", depth);
    
    parse_argument(argc, argv, "-solver_divide", solver_divide);
    print_info("Setting solver_divide to: ");
    print_value("%d\n", solver_divide);
    
    parse_argument(argc, argv, "-iso_divide", iso_divide);
    print_info("Setting iso_divide to: ");
    print_value("%d\n", iso_divide);
    
    parse_argument(argc, argv, "-point_weight", point_weight);
    print_info("Setting point_weight to: ");
    print_value("%f\n", point_weight);
    
    parse_argument(argc, argv, "-use_kd_tree", use_kd_tree);
    print_info("Setting kd_tree to: ");
    print_value("%d\n", use_kd_tree);
}

void parseGP(int argc, char** argv, float &resolution, int &padding_size, int &nearest_neighbor_no, int &max_binary_search_level, bool &use_kd_tree) {
    parse_argument(argc, argv, "-resolution", resolution);
    print_info("Using a resolution of: ");
    print_value("%d\n", resolution);
    
    parse_argument(argc, argv, "-padding_size", padding_size);
    print_info("Setting solver_divide to: ");
    print_value("%d\n", padding_size);
    
    parse_argument(argc, argv, "-nearest_neighbor_no", nearest_neighbor_no);
    print_info("Setting nearest_neighbor_no to: ");
    print_value("%d\n", nearest_neighbor_no);
    
    parse_argument(argc, argv, "-max_binary_search_level", max_binary_search_level);
    print_info("Setting max_binary_search_level to: ");
    print_value("%f\n", max_binary_search_level);
    
    parse_argument(argc, argv, "-use_kd_tree", use_kd_tree);
    print_info("Setting kd_tree to: ");
    print_value("%d\n", use_kd_tree);
}

int main(int argc, char** argv) {
    print_info("Compute the surface reconstruction of a point cloud. For more information, use: %s -h\n", argv[0]);

    if (argc < 3) {
        printHelp(argc, argv);
        return (-1);
    }

    // Parse the command line arguments for .ply files
    vector<int> ply_file_indices = parse_file_extension_argument(argc, argv, ".ply");
    vector<int> xyz_file_indices;
    if (ply_file_indices.size() != 1) {
        xyz_file_indices = parse_file_extension_argument(argc, argv, ".xyz");
        if (xyz_file_indices.size() != 1) {
            print_error("Need one input PLY or XYZ file to continue.\n");
            return (-1);
        }
    }

    vector<int> obj_file_indices = parse_file_extension_argument(argc, argv, ".obj");
    vector<int> stl_file_indices;
    if (obj_file_indices.size() != 1) {
        stl_file_indices = parse_file_extension_argument(argc, argv, ".stl");
        if (stl_file_indices.size() != 1) {
            print_error("Need one output OBJ or STL file to continue.\n");
            return (-1);
        }
    }

    // Command line parsing
    //parse method of reconstruction
    string method = default_method;
    parse_argument(argc, argv, "-method", method);
    print_info("Using method of: ");
    print_value("%s\n", method.c_str());
    PolygonMesh output;
    bool use_kd_tree = default_use_kd_tree;
    if (!method.compare(METHOD_OF_POISSON)) {
        int depth = default_depth;
        int solver_divide = default_solver_divide;
        int iso_divide = default_iso_divide;
        float point_weight = default_point_weight;
        parsePoisson(argc, argv, depth, solver_divide, iso_divide, point_weight, use_kd_tree);
        // Load the first file
        PointCloud<PointNormal>::Ptr cloud(new PointCloud<PointNormal>);
        createCloudWithNormals(ply_file_indices.size() == 1?argv[ply_file_indices[0]]:argv[xyz_file_indices[0]], *cloud, ply_file_indices.size() == 1);
        // Apply the Poisson surface reconstruction algorithm
        computePoisson (cloud, output, depth, solver_divide, iso_divide, point_weight, use_kd_tree);
    } else if (!method.compare(METHOD_OF_GRID_PROJECTION)) {
        float resolution = default_resolution;
        int padding_size = default_padding_size;
        int nearest_neighbor_no = default_nearest_neighbor_no;
        int max_binary_search_level = default_max_binary_search_level;
        parseGP(argc, argv, resolution, padding_size, nearest_neighbor_no, max_binary_search_level, use_kd_tree);
        // Load the first file
        PointCloud<PointNormal>::Ptr cloud(new PointCloud<PointNormal>);
        createCloudWithNormals(argv[ply_file_indices[0]], *cloud, ply_file_indices.size() == 1);
        //Apply the Grid Projection surface reconstruction algorithm
        computeGridProjection(cloud, output, resolution, padding_size, nearest_neighbor_no, max_binary_search_level, use_kd_tree);
    } else {
        print_error("Need valid method parameter: p or gp\n");
        return (-1);
    }
    // Save into the second file
    saveSurface (obj_file_indices.size() == 1?argv[obj_file_indices[0]]:argv[stl_file_indices[0]], output, obj_file_indices.size() == 1);
}