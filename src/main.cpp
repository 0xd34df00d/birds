#include <fstream>
#include <iostream>
#include <unordered_set>
#include <limits>
#include <boost/polygon/voronoi.hpp>

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

std::vector<Point_t> readPoints ()
{
	std::ifstream istr("data/п_1.txt");
	//std::ifstream istr("test.txt");
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

std::map<Point_t, std::vector<Point_t>> buildReachableMap (const std::vector<Point_t>& pts, const bp::voronoi_diagram<double>& vd)
{
	typedef std::decay<decltype (vd.edges ().front ().cell ())>::type Cell_t;
	std::map<Point_t, std::vector<Point_t>> point2reachable;
	for (const auto& edge : vd.edges ())
	{
		auto cell1 = edge.cell ();
		auto cell2 = edge.twin ()->cell ();

		const auto& pt1 = pts [cell1->source_index ()];
		const auto& pt2 = pts [cell2->source_index ()];

		if (distance (pt1, pt2) > distThreshold)
		{
			std::cout << "skipping" << std::endl;
			continue;
		}

		point2reachable [pt1].push_back (pt2);
	}
	return point2reachable;
}

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

enum class Pos
{
	Left,
	Right,
	Fwd,
	Back
};

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

int main ()
{
	const auto& pts = readPoints ();

	bp::voronoi_diagram<double> vd;
	bp::construct_voronoi (pts.begin (), pts.end (), &vd);

	const auto& point2reachable = buildReachableMap (pts, vd);

	std::vector<Point_t> hullPoints;
	const auto& start = getExtreme (pts);
	hullPoints.push_back (start);

	const auto last = hullPoints.back ();
	auto reachable = point2reachable.at (last);
	const auto& max = *std::max_element (reachable.begin (), reachable.end (),
			[&last] (const Point_t& p1, const Point_t& p2)
			{
				const auto c = classify (last, p1, p2);
				return c == Pos::Left || c == Pos::Fwd;
			});
	hullPoints.push_back (max);

	while (true)
	{
		const auto last = hullPoints.back ();
		const auto prelast = hullPoints [hullPoints.size () - 2];
		auto reachable = point2reachable.at (last);
		const auto& max = *std::max_element (reachable.begin (), reachable.end (),
				[&last, &prelast] (const Point_t& p1, const Point_t& p2)
				{
					if (p1 == prelast)
						return true;
					const auto c = classify (last, p1, p2);
					//std::cout << last << ": " << p1 << " v " << p2 << " " << static_cast<int> (c) << std::endl;
					if (c == Pos::Fwd)
						return true;
					if (c != Pos::Left)
						return false;

					if (classify (prelast, last, p1) == Pos::Left)
						return classify (prelast, last, p2) == Pos::Left;
					else
						return true;
				});
		hullPoints.push_back (max);

		std::cout << last << " -> " << max << std::endl;

		if (max == start)
			break;
	}

	std::vector<Segment_t> hullSegs;

	for (size_t i = 1; i < hullPoints.size (); ++i)
		hullSegs.push_back ({ hullPoints [i - 1], hullPoints [i] });

	bp::voronoi_diagram<double> skeleton;
	bp::construct_voronoi (hullSegs.begin (), hullSegs.end (), &skeleton);
}
