#include <fstream>
#include <iostream>
#include <unordered_set>
#include <limits>
#include <stdexcept>
#include <boost/polygon/voronoi.hpp>
#include <boost/polygon/segment_concept.hpp>

namespace bp = boost::polygon;

typedef bp::point_data<int> Point_t;
typedef bp::segment_data<Point_t::coordinate_type> Segment_t;

std::istream& operator>> (std::istream& istr, Point_t& p)
{
	Point_t::coordinate_type x = 0, y = 0;
	istr >> x >> y;
	p.x (x).y (y);
	return istr;
}

std::ostream& operator<< (std::ostream& ostr, const Point_t& p)
{
	ostr << "{ " << p.x () << "; " <<  p.y () << " }";
	return ostr;
}

struct PairHash
{
	template<typename T>
	size_t operator() (const std::pair<T*, T*>& p) const
	{
		size_t h1 = reinterpret_cast<size_t> (p.first);
		size_t h2 = reinterpret_cast<size_t> (p.second);
		return ((h1 << 16) | (h1 >> 16)) ^ h2;
	}
};

const double distThreshold = 30;

size_t distance (const Point_t& t1, const Point_t& t2)
{
	return std::sqrt (std::pow (t1.x () - t2.x (), 2) + std::pow (t1.y () - t2.y (), 2));
}

enum class Pos
{
	Left,
	Right,
	Fwd,
	Back
};

double dot (const Point_t& v0, const Point_t& v1)
{
	return v0.x () * v1.x () + v0.y () * v1.y ();
}

double absVec (const Point_t& v0, const Point_t& v1)
{
	return v0.x () * v1.y () - v0.y () * v1.x ();
}

Point_t operator- (const Point_t& p1, const Point_t& p2)
{
	return { p1.x () - p2.x (), p1.y () - p2.y () };
}

Pos classify (const Point_t& p0, const Point_t& p1, const Point_t& p)
{
	const auto c = 10 * absVec (p1 - p0, p - p0);
	const auto s = dot (p - p0, p1 - p0);

	if (c > 0)
		return Pos::Left;
	else if (c < 0)
		return Pos::Right;
	else if (s < 0)
		return Pos::Back;
	else
		return Pos::Fwd;
}

std::vector<Point_t> readPoints (const std::string& filename)
{
	std::ifstream istr (filename);
	int count = 0;
	istr >> count;
	std::vector<Point_t> pts;
	pts.reserve (count);
	for (int i = 0; i < count; ++i)
	{
		Point_t p;
		istr >> p;
		pts.push_back (p);
	}
	return pts;
}

Point_t getExtreme (const std::vector<Point_t>& points)
{
	Point_t result = points.front ();
	for (const auto& cand : points)
		if (cand.y () < result.y () ||
			(cand.y () == result.y () && cand.x () < result.x ()))
			result = cand;
	return result;
}

typedef std::map<Point_t, std::vector<Point_t>> ReachableMap_t;

ReachableMap_t buildReachableMap (const std::vector<Point_t>& pts, const bp::voronoi_diagram<double>& vd)
{
	typedef std::decay<decltype (vd.edges ().front ().cell ())>::type Cell_t;
	std::map<Point_t, std::vector<Point_t>> point2reachable;
	for (const auto& edge : vd.edges ())
	{
		auto cell1 = edge.cell ();
		auto cell2 = edge.twin ()->cell ();

		const auto& pt1 = pts [cell1->source_index ()];
		const auto& pt2 = pts [cell2->source_index ()];

		point2reachable [pt1].push_back (pt2);
	}
	return point2reachable;
}

std::list<Point_t> buildFullHull (const std::vector<Point_t>& pts, const bp::voronoi_diagram<double>& vd, const ReachableMap_t& point2reachable)
{
	std::list<Point_t> hullPoints;
	const auto& start = getExtreme (pts);
	hullPoints.push_back (start);

	while (true)
	{
		const auto last = hullPoints.back ();
		auto reachable = point2reachable.at (last);
		const auto& max = *std::max_element (reachable.begin (), reachable.end (),
				[&last] (const Point_t& p1, const Point_t& p2)
				{
					const auto c = classify (last, p1, p2);
					return c == Pos::Left || c == Pos::Fwd;
				});
		hullPoints.push_back (max);

		if (max == start)
			break;
	}
	return hullPoints;
}

Point_t intersect (const Point_t& p1, const Point_t& p2, ReachableMap_t& map)
{
	auto& v1 = map [p1];
	auto& v2 = map [p2];

	v1.erase (std::find (v1.begin (), v1.end (), p2));
	v2.erase (std::find (v2.begin (), v2.end (), p1));

	for (const auto& r1 : v1)
		if (std::find (v2.begin (), v2.end (), r1) != v2.end ())
			return r1;

	throw std::runtime_error ("intersection not found");
}

std::vector<Point_t> buildHull (const std::vector<Point_t>& pts, const bp::voronoi_diagram<double>& vd, ReachableMap_t point2reachable)
{
	auto hullPoints = buildFullHull (pts, vd, point2reachable);

	const int threshold = 10;

	auto prevPoint = hullPoints.front ();
	auto pos = ++hullPoints.begin ();
	while (pos != hullPoints.end ())
	{
		auto point = *pos;
		try
		{
			if (distance (prevPoint, point) >= threshold)
			{
				pos = hullPoints.insert (pos, intersect (prevPoint, point, point2reachable));
				continue;
			}
		}
		catch (...)
		{
		}

		prevPoint = point;
		++pos;
	}

	std::vector<Point_t> result;
	result.reserve (hullPoints.size ());
	std::copy (hullPoints.begin (), hullPoints.end (), std::back_inserter (result));
	return result;
}

void printPoints (const std::vector<Point_t>& pts, const std::string& filename)
{
	std::ofstream ostr (filename);
	for (const auto& pt : pts)
		ostr << pt.x () << " " << pt.y () << "\n";
}

int main (int argc, char **argv)
{
	const std::string filename ("data/п_1.txt");
	const auto& pts = readPoints (filename);

	bp::voronoi_diagram<double> vd;
	bp::construct_voronoi (pts.begin (), pts.end (), &vd);

	const auto& point2reachable = buildReachableMap (pts, vd);

	const auto& hullPoints = buildHull (pts, vd, point2reachable);

	std::vector<Segment_t> hullSegs;

	for (size_t i = 1; i < hullPoints.size (); ++i)
		hullSegs.push_back ({ hullPoints [i - 1], hullPoints [i] });

	printPoints (hullPoints, filename + ".hull");

	bp::voronoi_diagram<double> skeleton;
	bp::construct_voronoi (hullSegs.begin (), hullSegs.end (), &skeleton);

	std::ofstream ostr (filename + ".skel");
	for (const auto& edge : skeleton.edges ())
	{
		if (!edge.is_finite () || !edge.is_primary ())
			continue;

		const auto& v0 = *edge.vertex0 (), v1 = *edge.vertex1 ();
		const Segment_t edgeSeg ({ v0.x (), v0.y () }, { v1.x (), v1.y () });

		if (std::any_of (hullSegs.begin (), hullSegs.end (),
				[&edgeSeg] (const Segment_t& seg)
				{
					return bp::intersects (seg, edgeSeg);
				}))
			continue;

		ostr << edge.vertex0 ()->x () << " " << edge.vertex0 ()->y () << "\n" << edge.vertex1 ()->x () << " " << edge.vertex1 ()->y () << "\n";
	}
}
