#include "image.h"
#include <future>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

namespace fs = boost::filesystem;

enum class ImgType
{
	Bird,
	Fish
};

struct LearnInfo
{
	Image_ptr Img_;
	ImgType Type_;
};

int main (int argc, char **argv)
{
	auto current = fs::current_path ();
	current /= "data";

	std::vector<fs::directory_entry> entries;
	std::copy (fs::directory_iterator (current), fs::directory_iterator (), std::back_inserter (entries));

	std::vector<LearnInfo> learnData;
	learnData.reserve (entries.size ());

#pragma omp parallel for schedule (dynamic, 6)
	for (size_t i = 0; i < entries.size (); ++i)
	{
		const auto entry = entries [i];

		const auto path = entry.path ();
		if (path.extension () != ".txt")
			continue;

		const auto leafStr = path.leaf ().string ();
		ImgType type = ImgType::Bird;
		if (leafStr.find ("п") == 0)
			type = ImgType::Bird;
		else if (leafStr.find ("р") == 0)
			type = ImgType::Fish;
		else
			continue;

		const auto str = path.string ();
		Image_ptr img (new Image (str));
		img->PrintPseudoHull ();
		img->PrintSkeleton ();
#pragma omp critical
		{
			learnData.push_back ({ img, type });
		}
	}
}
