#include "glwidget.h"
#include <QtDebug>

void GLWidget::build (const QString& file_path)
{
	// Clear all containers.
	clear();

	// Read data.
	read_data (file_path);

	// No data, don't proceed.
	if (!brect_initialized_) {
		return;
	}

	// Construct bounding rectangle.
	construct_brect();

	// Construct voronoi diagram.
	construct_voronoi (
		point_data_.begin(), point_data_.end(),
		segment_data_.begin(), segment_data_.end(),
		&vd_);

	// Color exterior edges.
	for (const_edge_iterator it = vd_.edges().begin();
			it != vd_.edges().end(); ++it) {
		if (!it->is_finite()) {
			color_exterior (& (*it));
		}
	}

	typedef decltype (vd_.edges () [0].cell ()) Cell_t;
	QSet<QPair<Cell_t, Cell_t>> traversed;
	for (const auto& edge : vd_.edges ())
	{
		auto cell1 = edge.cell ();
		auto cell2 = edge.twin ()->cell ();
		auto p = qMakePair (cell1, cell2);
		if (traversed.contains (p))
			continue;

		traversed << p << qMakePair (cell2, cell1);

		const auto& pt1 = point_data_ [cell1->source_index ()];
		const auto& pt2 = point_data_ [cell2->source_index ()];

		hull_data_.push_back ({ pt1, pt2 });
	}

	// Update view port.
	update_view_port();
}

namespace
{
	template<typename T, typename U>
	void drawSegments (const T& segs, const U& shift)
	{
		glBegin (GL_LINES);

		for (std::size_t i = 0; i < segs.size(); ++i) {
			auto lp = low (segs[i]);
			lp = deconvolve (lp, shift);
			glVertex2f (lp.x(), lp.y());
			auto hp = high (segs[i]);
			hp = deconvolve (hp, shift);
			glVertex2f (hp.x(), hp.y());
		}
	}
}

void GLWidget::draw_segments()
{
	glColor3f (0.0f, 0.5f, 1.0f);
	glLineWidth (2.7f);
	drawSegments (segment_data_, shift_);
	glEnd();
}

void GLWidget::draw_hull()
{
	// Draw input segments.
	glColor3f (0.0f, 1.0f, 0.0f);
	glLineWidth (3.f);
	drawSegments (hull_data_, shift_);
	glEnd();
}
