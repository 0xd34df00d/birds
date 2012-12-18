#include "image.h"

namespace
{
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

	Point_t getExtreme (const std::vector<Point_t>& points)
	{
		Point_t result = points.front ();
		for (const auto& cand : points)
			if (cand.y () < result.y () ||
				(cand.y () == result.y () && cand.x () < result.x ()))
				result = cand;
		return result;
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

	Point_t intersect (const Point_t& p1, const Point_t& p2, Image::ReachableMap_t& map)
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

	void printPoints (const std::vector<Point_t>& pts, const std::string& filename)
	{
		std::ofstream ostr (filename);
		for (const auto& pt : pts)
			ostr << pt.x () << " " << pt.y () << "\n";
	}
}

Image::Image (const std::string& filename)
: Filename_ (filename)
, SourcePoints_ (readPoints (filename))
{
	bp::construct_voronoi (SourcePoints_.begin (), SourcePoints_.end (), &SourceVD_);

	BuildReachableMap ();
	BuildFullHull ();
	PseudoHull_ = BuildPseudoHull ();
	BuildPseudoHullSegs ();
	BuildSkeleton ();
}

void Image::PrintPseudoHull () const
{
	printPoints (PseudoHull_, Filename_ + ".hull");
}

void Image::PrintSkeleton () const
{
	std::ofstream ostr (Filename_ + ".skel");
	for (const auto& edge : SkeletonVD_.edges ())
	{
		if (!edge.is_finite () || !edge.is_primary ())
			continue;

		const auto& v0 = *edge.vertex0 (), v1 = *edge.vertex1 ();
		const Segment_t edgeSeg ({ v0.x (), v0.y () }, { v1.x (), v1.y () });

		if (std::any_of (PseudoHullSegs_.begin (), PseudoHullSegs_.end (),
				[&edgeSeg] (const Segment_t& seg)
				{
					return bp::intersects (seg, edgeSeg);
				}))
			continue;

		ostr << edge.vertex0 ()->x () << " " << edge.vertex0 ()->y () << "\n" << edge.vertex1 ()->x () << " " << edge.vertex1 ()->y () << "\n";
	}
}

void Image::BuildReachableMap ()
{
	typedef std::decay<decltype (SourceVD_.edges ().front ().cell ())>::type Cell_t;
	for (const auto& edge : SourceVD_.edges ())
	{
		auto cell1 = edge.cell ();
		auto cell2 = edge.twin ()->cell ();

		const auto& pt1 = SourcePoints_ [cell1->source_index ()];
		const auto& pt2 = SourcePoints_ [cell2->source_index ()];

		FullReachable_ [pt1].push_back (pt2);
	}
}

void Image::BuildFullHull ()
{
	const auto& start = getExtreme (SourcePoints_);
	FullHull_.push_back (start);

	while (true)
	{
		const auto last = FullHull_.back ();
		auto reachable = FullReachable_.at (last);
		if (FullHull_.size () > 1)
			reachable.erase (std::find (reachable.begin (), reachable.end (), *(++FullHull_.rbegin ())));
		const auto& max = *std::max_element (reachable.begin (), reachable.end (),
				[&last] (const Point_t& p1, const Point_t& p2)
				{
					const auto c = classify (last, p1, p2);
					return c == Pos::Left || c == Pos::Fwd;
				});
		FullHull_.push_back (max);

		if (max == start)
			break;
	}
}

std::vector<Point_t> Image::BuildPseudoHull () const
{
	const int threshold = 10;

	auto hullPoints = FullHull_;
	auto point2reachable = FullReachable_;

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

void Image::BuildPseudoHullSegs ()
{
	for (size_t i = 1; i < PseudoHull_.size (); ++i)
		PseudoHullSegs_.push_back ({ PseudoHull_ [i - 1], PseudoHull_ [i] });
}

void Image::BuildSkeleton ()
{
	bp::construct_voronoi (PseudoHullSegs_.begin (), PseudoHullSegs_.end (), &SkeletonVD_);
}
