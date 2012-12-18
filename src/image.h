#pragma once

#include <fstream>
#include <iostream>
#include <unordered_set>
#include <limits>
#include <stdexcept>
#include <memory>
#include <boost/polygon/voronoi.hpp>

namespace bp = boost::polygon;

typedef bp::point_data<int> Point_t;
typedef bp::segment_data<Point_t::coordinate_type> Segment_t;

class Image
{
	const std::string Filename_;
	const std::vector<Point_t> SourcePoints_;

	bp::voronoi_diagram<double> SourceVD_;

public:
	typedef std::map<Point_t, std::vector<Point_t>> ReachableMap_t;
private:
	ReachableMap_t FullReachable_;

	std::list<Point_t> FullHull_;
	std::vector<Point_t> PseudoHull_;

	std::vector<Segment_t> PseudoHullSegs_;

	bp::voronoi_diagram<double> SkeletonVD_;
public:
	Image (const std::string&);

	void PrintPseudoHull () const;
	void PrintSkeleton () const;
private:
	void BuildReachableMap ();

	void BuildFullHull ();
	std::vector<Point_t> BuildPseudoHull () const;
	void BuildPseudoHullSegs ();

	void BuildSkeleton ();
};

typedef std::shared_ptr<Image> Image_ptr;
