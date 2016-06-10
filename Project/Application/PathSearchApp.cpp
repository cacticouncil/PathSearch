// PathPlannerLab.cpp : Defines the entry point for the path planner application.
// Code by Cromwell D. Enage
// October 2010
#include "../platform.h"

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cassert>

#include <string>
#include <iostream>
#include <algorithm>
#include <deque>
#include <map>
#include <functional>

#include <windows.h>

#include "PathSearchUtility.h"
#include "PathSearchApp.h"

#ifdef _MSC_VER
#include <io.h>
#include <fcntl.h>
#endif

std::vector<fullsail_ai::Tile const*> tempSolution;

PathSearchGlobals* PathSearchGlobals::instance_ = 0;

PathSearchGlobals::PathSearchGlobals() : min_radius_(4.0), flags_(0)
{
}

PathSearchGlobals::~PathSearchGlobals()
{
}

PathSearchGlobals* PathSearchGlobals::getInstance()
{
	if (instance_ == 0)
	{
		instance_ = new PathSearchGlobals();
	}

	return instance_;
}

void PathSearchGlobals::deleteInstance()
{
	if (instance_ != 0)
	{
		delete instance_;
		instance_ = 0;
	}
}

using namespace std;
using namespace fullsail_ai;
using namespace algorithms;

void setBounds(double const h_radius, int const row_count, int const column_count,
               POINT const& offset, int grid_width, int grid_height,
               int& row_start, int& row_end, int& column_start, int& column_end)
{
	double const inv_radius = sqrt(3.0) / h_radius;

	row_start = (static_cast<int>(-offset.y * inv_radius) - 1) / 3;
	column_start = (static_cast<int>(-offset.x / h_radius) - 1) >> 1;

	if (column_start < 0)
	{
		column_start = 0;
	}

	row_end = static_cast<int>((grid_height - offset.y) * inv_radius) / 3;

	if (row_count < row_end)
	{
		row_end = row_count;
	}

	column_end = static_cast<int>(((grid_width - offset.x) >> 1) / h_radius);

	if (column_count < column_end)
	{
		column_end = column_count;
	}
}

void drawGrid(TileMap const& tile_map, POINT const& offset, int grid_width, int grid_height,
              HDC device_context_handle)
{
	double const radius = tile_map.getTileRadius();

	POINT hexagon[6];

	hexagon[0].x = hexagon[3].x = 0;
	hexagon[4].x = hexagon[5].x = -(hexagon[1].x = hexagon[2].x = static_cast<LONG>(radius));

	if (hexagon[4].x != radius)
	{
		--hexagon[1].x;
		--hexagon[2].x;
		++hexagon[4].x;
		++hexagon[5].x;
	}
	else if (radius < 6.0)
	{
		++hexagon[1].x;
		++hexagon[2].x;
		--hexagon[4].x;
		--hexagon[5].x;
	}

	hexagon[1].y = hexagon[5].y = -(
		hexagon[2].y = hexagon[4].y = static_cast<LONG>(radius / sqrt(3.0))
	);
	hexagon[0].y = -(hexagon[3].y = hexagon[2].y << 1);

	for (unsigned int i = 0; i < 6; ++i)
	{
		hexagon[i].x += offset.x;
		hexagon[i].y += offset.y;
	}

	if (HRGN hex_region = CreatePolygonRgn(hexagon, 6, WINDING))
	{
		int row_start;
		int row_end;
		int column_start;
		int column_end;

		setBounds(radius, tile_map.getRowCount(), tile_map.getColumnCount(), offset,
		          grid_width, grid_height, row_start, row_end, column_start, column_end);

		HBRUSH brush_handle[16];
		HBRUSH black_brush_handle = reinterpret_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));
		unsigned char tile_weight;

		for (unsigned char i = 0; i < 16; ++i)
		{
			tile_weight = 255 - (i << 4);
			brush_handle[i] = CreateSolidBrush(RGB(tile_weight, tile_weight, tile_weight));
		}

		Tile const* tile;
		int x_offset;
		int y_offset;
		int row;
		int column;

		for (row = row_start; row < row_end; ++row)
		{
			for (column = column_start; column < column_end; ++column)
			{
				tile = tile_map.getTile(row, column);
				x_offset = static_cast<int>(tile->getXCoordinate());
				y_offset = static_cast<int>(tile->getYCoordinate());

				if (OffsetRgn(hex_region, x_offset, y_offset) != ERROR)
				{
					tile_weight = tile->getWeight();
					if (tile_weight != 0)
					{
						if (15 < tile_weight)
						{
							tile_weight = 15;
						}

						FillRgn(device_context_handle, hex_region, brush_handle[tile_weight]);
					}
					else
					{
						FillRgn(device_context_handle, hex_region, black_brush_handle);
					}

					OffsetRgn(hex_region, -x_offset, -y_offset);
				}
			}
		}

		for (unsigned char i = 0; i < 16; ++i)
		{
			DeleteObject(brush_handle[i]);
		}

		DeleteObject(hex_region);
	}
}

void drawEndpoint(int center_x, int center_y, int length, HBRUSH brush_handle,
                  HDC device_context_handle)
{
	int x2 = length >> 1;
	int x1 = center_x - x2;
	int y1 = center_y - length;
	RECT rectangle;

	x2 += center_x;
	rectangle.left = x1 + 1;
	rectangle.right = x2;
	rectangle.top = y1 + 1;
	rectangle.bottom = center_y;
	FillRect(device_context_handle, &rectangle, brush_handle);

	if (MoveToEx(device_context_handle, x1, center_y + length, 0))
	{
		LineTo(device_context_handle, x1, y1);
		LineTo(device_context_handle, x2, y1);
		LineTo(device_context_handle, x2, center_y);
		LineTo(device_context_handle, x1, center_y);
	}
}

void displayEndpoints(Tile const* start_tile, Tile const* goal_tile, POINT const& offset,
                      int half_length, HBRUSH start_brush_handle, HBRUSH goal_brush_handle,
                      HDC device_context_handle)
{
	LONG const offset_x = offset.x;
	LONG const offset_y = offset.y;

	drawEndpoint(static_cast<int>(start_tile->getXCoordinate() + offset_x),
	             static_cast<int>(start_tile->getYCoordinate() + offset_y),
	             half_length, start_brush_handle, device_context_handle);
	drawEndpoint(static_cast<int>(goal_tile->getXCoordinate() + offset_x),
	             static_cast<int>(goal_tile->getYCoordinate() + offset_y),
	             half_length, goal_brush_handle, device_context_handle);
}

PathSearchInterface::~PathSearchInterface()
{
}

BOOL PathSearchInterface::shouldEnableSetStart() const
{
	return FALSE;
}

bool PathSearchInterface::updateStart()
{
	return false;
}

BOOL PathSearchInterface::shouldEnableSetGoal() const
{
	return FALSE;
}

bool PathSearchInterface::updateGoal()
{
	return false;
}

BOOL PathSearchInterface::shouldEnableRun() const
{
	if (isInitializableByKey() || isRunnableByKey())
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

BOOL PathSearchInterface::shouldEnableTimeRun() const
{
	return FALSE;
}

void PathSearchInterface::timeSearch()
{
}

void PathSearchInterface::checkSolution(HWND window_handle) const
{
}

bool PathSearchInterface::needsFullRedraw() const
{
	return true;
}

void PathSearchInterface::endRedrawSearchProgress(POINT const& offset, int width, int height,
                                                   HDC device_context_handle) const
{
	displaySearchProgress(offset, width, height, device_context_handle);
}

GroundUpPathSearch::GroundUpPathSearch(TileMap& tiles)
	: search_(), tile_map_(tiles), start_tile_(0), goal_tile_(0), frequency_(), elapsed_time_(0.0)
	, iteration_count_(0), start_row_(0), start_column_(0), goal_row_(0), goal_column_(0), myTimeStep(0), myFastTimeStep(5000), myNumberofRounds(1)
	, is_initializable_(true)
{
	QueryPerformanceFrequency(&frequency_);
}

bool GroundUpPathSearch::isReady() const
{
	return tile_map_.getRowCount() && tile_map_.getColumnCount();
}

void GroundUpPathSearch::resetParameters()
{
	start_row_ = start_column_ = goal_row_ = goal_column_ = myTimeStep = 0;
	myFastTimeStep = 5000;
	myNumberofRounds = 1;
	start_tile_ = goal_tile_ = 0;
}

void GroundUpPathSearch::shutdownSearch()
{
	search_.shutdown();
}

bool GroundUpPathSearch::read(basic_ifstream<TCHAR>& input_stream)
{
#if OVERRIDE_DEFAULT_STARTING_DATA 

	start_tile_ = tile_map_.getTile(start_row_ = DEFAULT_START_ROW, start_column_ = DEFAULT_START_COL);
	goal_tile_ = tile_map_.getTile(goal_row_ = DEFAULT_GOAL_ROW,
	                               goal_column_ = DEFAULT_GOAL_COL);

#else

	start_tile_ = tile_map_.getTile(start_row_ = 0, start_column_ = 0);
	goal_tile_ = tile_map_.getTile(goal_row_ = tile_map_.getRowCount() - 1,
	                               goal_column_ = tile_map_.getColumnCount() - 1);

#endif
	return true;
}

void GroundUpPathSearch::initialize()
{
	search_.initialize(&tile_map_);
}

int GroundUpPathSearch::getInputCount() const
{
	return 7;
}

void GroundUpPathSearch::displayInput(NMLVDISPINFO* list_view_display_info) const
{
	LVITEM& item = list_view_display_info->item;

	switch (item.iItem)
	{
		case 0:
		{
			if (item.iSubItem)
			{
				_stprintf(item.pszText, _T("%s"), _T("Start Row"));
			}
			else
			{
				_stprintf(item.pszText, _T("%i"), start_row_);
			}

			break;
		}

		case 1:
		{
			if (item.iSubItem)
			{
				_stprintf(item.pszText, _T("%s"), _T("Start Column"));
			}
			else
			{
				_stprintf(item.pszText, _T("%i"), start_column_);
			}

			break;
		}

		case 2:
		{
			if (item.iSubItem)
			{
				_stprintf(item.pszText, _T("%s"), _T("Goal Row"));
			}
			else
			{
				_stprintf(item.pszText, _T("%i"), goal_row_);
			}

			break;
		}

		case 3:
		{
			if (item.iSubItem)
			{
				_stprintf(item.pszText, _T("%s"), _T("Goal Column"));
			}
			else
			{
				_stprintf(item.pszText, _T("%i"), goal_column_);
			}

			break;
		}

		case 4:
		{
			if (item.iSubItem)
			{
				_stprintf(item.pszText, _T("%s"), _T("Regular Run Time Step(ms)"));
			}
			else
			{
				_stprintf(item.pszText, _T("%i"), myTimeStep);
			}

			break;
		}

		case 5:
		{
			if (item.iSubItem)
			{
				_stprintf(item.pszText, _T("%s"), _T("Fast Run Time Step(ms)"));
			}
			else
			{
				_stprintf(item.pszText, _T("%i"), myFastTimeStep);
			}

			break;
		}

		case 6:
		{
			if (item.iSubItem)
			{
				_stprintf(item.pszText, _T("%s"), _T("Number of rounds"));
			}
			else
			{
				_stprintf(item.pszText, _T("%i"), myNumberofRounds);
			}

			break;
		}
	}
}

bool GroundUpPathSearch::updateInput(NMLVDISPINFO const* list_view_display_info)
{
	LVITEM const& item = list_view_display_info->item;

	switch (item.iItem)
	{
		case 0:
		{
			int row = 0;

			if (
			    _stscanf(item.pszText, _T("%i"), &row)
			 && (0 <= row)
			 && (row < tile_map_.getRowCount())
			 && tile_map_.getTile(row, start_column_)->getWeight()
			)
			{
				start_row_ = tile_map_.getRowCount();

				if (row < start_row_)
				{
					start_row_ = row;
				}
				else
				{
					--start_row_;
				}

				start_tile_ = tile_map_.getTile(start_row_, start_column_);
				return true;
			}

			break;
		}

		case 1:
		{
			int column = 0;

			if (
			    _stscanf(item.pszText, _T("%i"), &column)
			 && (0 <= column)
			 && (column < tile_map_.getColumnCount())
			 && tile_map_.getTile(start_row_, column)->getWeight()
			)
			{
				start_column_ = tile_map_.getColumnCount();

				if (column < start_column_)
				{
					start_column_ = column;
				}
				else
				{
					--start_column_;
				}

				start_tile_ = tile_map_.getTile(start_row_, start_column_);
				return true;
			}

			break;
		}

		case 2:
		{
			int row = 0;

			if (
			    _stscanf(item.pszText, _T("%i"), &row)
			 && (0 <= row)
			 && (row < tile_map_.getRowCount())
			 && tile_map_.getTile(row, goal_column_)->getWeight()
			)
			{
				goal_row_ = tile_map_.getRowCount();

				if (row < goal_row_)
				{
					goal_row_ = row;
				}
				else
				{
					--goal_row_;
				}

				goal_tile_ = tile_map_.getTile(goal_row_, goal_column_);
				return true;
			}

			break;
		}

		case 3:
		{
			int column = 0;

			if (
			    _stscanf(item.pszText, _T("%i"), &column)
			 && (0 <= column)
			 && (column < tile_map_.getColumnCount())
			 && tile_map_.getTile(goal_row_, column)->getWeight()
			)
			{
				goal_column_ = tile_map_.getColumnCount();

				if (column < goal_column_)
				{
					goal_column_ = column;
				}
				else
				{
					--goal_column_;
				}

				goal_tile_ = tile_map_.getTile(goal_row_, goal_column_);
				return true;
			}

			break;
		}

		case 4:
			{
				_stscanf(item.pszText, _T("%i"), &myTimeStep);
					break;
			}

		case 5:
			{
				_stscanf(item.pszText, _T("%i"), &myFastTimeStep);
				break;
			}

		case 6:
			{
				_stscanf(item.pszText, _T("%i"), &myNumberofRounds);
				break;
			}
	}

	return false;
}

void GroundUpPathSearch::resetSearch()
{
	if(mybHasEntered)
		search_.exit();

		tempSolution.clear();

	tile_map_.resetTileDrawing();
	elapsed_time_ = 0.0;
	iteration_count_ = 0;
	is_initializable_ = true;
	mybHasEntered = false;
}

bool GroundUpPathSearch::isInitializableByKey() const
{
	return is_initializable_;
}

void GroundUpPathSearch::initializeSearch()
{
	elapsed_time_ = 0.0;
	iteration_count_ = 0;
	if(!mybTimeSearch)
	{
		search_.enter(start_row_, start_column_, goal_row_, goal_column_);
		mybHasEntered  = true;
	}
	is_initializable_ = false;

}

bool GroundUpPathSearch::isRunnableByKey() const
{
	return !is_initializable_ && !search_.isDone()/*search_.getSolution().empty()*/;
}

void GroundUpPathSearch::runSearch(void/*long long timeslice*/)
{
	if(!search_.isDone())
	{
		search_.update(static_cast<long>(myTimeStep/*timeslice*/));
		++iteration_count_;
	}
}

void GroundUpPathSearch::stepSearch(void)
{
	if(!search_.isDone())
	{
		search_.update(static_cast<long>(0/*timeslice*/));
		++iteration_count_;
	}
}

BOOL GroundUpPathSearch::shouldEnableTimeRun() const
{
	return isReady() ? TRUE : FALSE;
}

using namespace std::placeholders;
void GroundUpPathSearch::timeSearch()
{
	LARGE_INTEGER time_start;
	LARGE_INTEGER time_end;

	mybTimeSearch = true;

	initializeSearch();

	tempSolution.clear();
	mybHasEntered = true;
	QueryPerformanceCounter(&time_start);
	for(unsigned int i = 0; i < myNumberofRounds; i++)
	{
		search_.enter(start_row_, start_column_, goal_row_, goal_column_);
		search_.update(myFastTimeStep);//Run for this long or else
		tempSolution = search_.getSolution();//don't like this but it solves functionality
		search_.exit();
	}

	QueryPerformanceCounter(&time_end);
	mybHasEntered = false;

	elapsed_time_ = static_cast<double>(
		time_end.QuadPart - time_start.QuadPart
		) / frequency_.QuadPart;
	std::cout << "average elapsed time: " << elapsed_time_/(double)myNumberofRounds << std::endl;

	if(search_.isDone())
	{
		if(tempSolution.size())
		{
			if (tempSolution.back() != start_tile_)
			{
				MessageBox(NULL,
					_T("The first tile is not the start!"),
					_T("Solution Checker"), MB_OK);
			}

			if (tempSolution.front() != goal_tile_)
			{
				MessageBox(NULL, _T("The last tile is not the goal!"),
					_T("Solution Checker"), MB_OK);
			}

			deque<double> v1, v2;
			std::size_t i = tempSolution.size();
			double dare = tile_map_.getTileRadius() * (1 << 1);
			double down = dare * dare;

			dare = down + 0.00001;
			down -= 0.00001;

			while (--i)
			{
				v1.clear();
				v2.clear();
				v1.push_back(tempSolution[i - 1]->getYCoordinate());
				v2.push_front(tempSolution[i]->getXCoordinate());
				v1.push_back(tempSolution[i - 1]->getXCoordinate());
				v2.push_front(tempSolution[i]->getYCoordinate());
				transform(v1.begin(), v1.end(), v2.begin(), v1.begin(), minus<double>());
				transform(v1.begin(), v1.end(), v2.begin(), [](double input) -> double { return pow(input, 2); });
				v1.back() = v2.back() + v2.front();

				if (dare < v1.back() || v1.back() < down)
				{
					MessageBox(NULL, _T("A node is not next to its parent!"),
						_T("Solution Checker"), MB_OK);
				}
			}
		}
		else/* if(!mybTimeSearch) *///!done
		{
			MessageBox(NULL,
				_T("isDone() returned true and getSolution() returned a vector size 0"),
				_T("Solution Checker"), MB_OK);
		}
	}
	else
	{
		MessageBox(NULL,
			_T("Either Update() returned without isDone() returning true \n            or           \nUpdate() is taking way too long!"),
			_T("Solution Checker"), MB_OK);
		resetSearch();
	}

}

void GroundUpPathSearch::checkSolution(HWND window_handle) const
{
	if (search_.isDone())
	{
		tempSolution = search_.getSolution();
		if(tempSolution.size())
		{
			if (tempSolution.back() != start_tile_)
			{
				MessageBox(window_handle,
					_T("The first tile is not the start!"),
					_T("Solution Checker"), MB_OK);
			}

			if (tempSolution.front() != goal_tile_)
			{
				MessageBox(window_handle, _T("The last tile is not the goal!"),
					_T("Solution Checker"), MB_OK);
			}

			deque<double> v1, v2;
			std::size_t i = tempSolution.size();
			double dare = tile_map_.getTileRadius() * (1 << 1);
			double down = dare * dare;

			dare = down + 0.00001;
			down -= 0.00001;

			while (--i)
			{
				v1.clear();
				v2.clear();
				v1.push_back(tempSolution[i - 1]->getYCoordinate());
				v2.push_front(tempSolution[i]->getXCoordinate());
				v1.push_back(tempSolution[i - 1]->getXCoordinate());
				v2.push_front(tempSolution[i]->getYCoordinate());
				transform(v1.begin(), v1.end(), v2.begin(), v1.begin(), minus<double>());
				transform(v1.begin(), v1.end(), v2.begin(), [](double input) -> double { return pow(input, 2); });
				v1.back() = v2.back() + v2.front();

				if (dare < v1.back() || v1.back() < down)
				{
					MessageBox(NULL, _T("A node is not next to its parent!"),
						_T("Solution Checker"), MB_OK);
				}
			}
		}
		else/* if(!mybTimeSearch) *///!done
		{
			MessageBox(NULL,
				_T("isDone() returned true and getSolution() returned a vector size 0"),
				_T("Solution Checker"), MB_OK);
		}
	}
	
}

void GroundUpPathSearch::displaySearchProgress(POINT const& offset, int width, int height,
                                                HDC device_context_handle) const
{
	if (!isReady())
	{
		return;
	}

	double const tile_radius = tile_map_.getTileRadius();
	int large_node_radius = static_cast<int>(tile_radius * 0.75);
	HBRUSH start_brush_handle = CreateSolidBrush(RGB(255, 0, 0));
	HBRUSH goal_brush_handle = CreateSolidBrush(RGB(0, 255, 0));

	displayEndpoints(start_tile_, goal_tile_, offset, large_node_radius,
	                 start_brush_handle, goal_brush_handle, device_context_handle);
	DeleteObject(goal_brush_handle);
	DeleteObject(start_brush_handle);

	Tile const* tile;
	int x, y, column;

	for (int row = 0; row < tile_map_.getRowCount(); ++row)
	{
		for (column = 0; column < tile_map_.getColumnCount(); ++column)
		{
			tile = tile_map_.getTile(row, column);
			x = static_cast<int>(offset.x + tile->getXCoordinate());
			y = static_cast<int>(offset.y + tile->getYCoordinate());

			if (unsigned int fill = tile->getFill())
			{
				int const radius = static_cast<int>(tile_radius * 0.75);
				unsigned int outline = tile->getOutline();
				HPEN outline_pen_handle
					= outline
					? CreatePen(PS_SOLID, 2, static_cast<COLORREF>(outline))
					: 0;
				HGDIOBJ old_pen_handle
					= outline_pen_handle
					? SelectObject(device_context_handle, outline_pen_handle)
					: 0;
				HBRUSH fill_brush_handle = CreateSolidBrush(static_cast<COLORREF>(fill));
				HGDIOBJ old_brush_handle = SelectObject(device_context_handle, fill_brush_handle);

				BeginPath(device_context_handle);
				MoveToEx(device_context_handle, x + radius, y, 0);
				AngleArc(device_context_handle, x, y, radius, 0.0f, 360.0f);
				EndPath(device_context_handle);
				StrokeAndFillPath(device_context_handle);
				SelectObject(device_context_handle, old_brush_handle);
				DeleteObject(fill_brush_handle);

				if (outline_pen_handle)
				{
					SelectObject(device_context_handle, old_pen_handle);
					DeleteObject(outline_pen_handle);
				}
			}

			if (unsigned int marker = tile->getMarker())
			{
				int const radius = static_cast<int>(tile_radius * 0.5);
				HBRUSH marker_brush_handle = CreateSolidBrush(static_cast<COLORREF>(marker));
				HGDIOBJ old_brush_handle = SelectObject(
				    device_context_handle
				  , marker_brush_handle
				);

				BeginPath(device_context_handle);
				MoveToEx(device_context_handle, x + radius, y, 0);
				AngleArc(device_context_handle, x, y, radius, 0.0f, 360.0f);
				EndPath(device_context_handle);
				StrokeAndFillPath(device_context_handle);
				SelectObject(device_context_handle, old_brush_handle);
				DeleteObject(marker_brush_handle);
			}
		}
	}

	if (/*!path.empty()*/search_.isDone())
	{
		vector<Tile const*> const& path = tempSolution;
		if(path.size())
		{
			RECT rectangle;
			int small_node_radius = static_cast<int>(tile_radius * 0.5);
			HBRUSH path_brush_handle = CreateSolidBrush(RGB(127, 255, 127));
			HGDIOBJ old_brush_handle = SelectObject(device_context_handle, path_brush_handle);

			for (size_t index = 0; index < path.size(); ++index)
			{
				tile = path[index];
				x = static_cast<int>(offset.x + tile->getXCoordinate());
				y = static_cast<int>(offset.y + tile->getYCoordinate());
				rectangle.left = x - small_node_radius;
				rectangle.right = x + small_node_radius;
				rectangle.top = y - small_node_radius;
				rectangle.bottom = y + small_node_radius;
				FillRect(device_context_handle, &rectangle, path_brush_handle);
			}

			SelectObject(device_context_handle, old_brush_handle);
			DeleteObject(path_brush_handle);
		}
		/*}
		else
		{
			MessageBox(NULL,
				_T("isDone() returned true and getSolution() returned a vector size 0"),
				_T("Solution Checker"), MB_OK);
		}*/
	}

	for (int row = 0; row < tile_map_.getRowCount(); ++row)
	{
		for (column = 0; column < tile_map_.getColumnCount(); ++column)
		{
			tile = tile_map_.getTile(row, column);
			
			//Get the head of the singly linked list
			const std::vector<pair<const Tile*, unsigned>> lines = tile->getLines();
			
			for (auto lineSet : lines)
			{
				const Tile* destination = lineSet.first;
				unsigned lineColor = lineSet.second;

				x = static_cast<int>(offset.x + tile->getXCoordinate());
				y = static_cast<int>(offset.y + tile->getYCoordinate());

				HPEN line_pen_handle = CreatePen(
					PS_SOLID
					, 3
					, static_cast<COLORREF>(lineColor)
					);
				HGDIOBJ old_pen_handle = SelectObject(device_context_handle, line_pen_handle);

				MoveToEx(device_context_handle, x, y, 0);
				x = static_cast<int>(offset.x + destination->getXCoordinate());
				y = static_cast<int>(offset.y + destination->getYCoordinate());
				LineTo(device_context_handle, x, y);
				SelectObject(device_context_handle, old_pen_handle);
				DeleteObject(line_pen_handle);
			}
		}
	}
}

void GroundUpPathSearch::beginRedrawSearchProgress(POINT const& offset, int width, int height,
                                                    HDC device_context_handle) const
{
	drawGrid(tile_map_, offset, width, height, device_context_handle);
}

BOOL onOkCancelCommand(HWND dialog_handle, WPARAM w_param)
{
	switch (LOWORD(w_param))
	{
		case IDOK:
		case IDCANCEL:
		{
			EndDialog(dialog_handle, LOWORD(w_param));
			return TRUE;
		}
	}

	return FALSE;
}

// Message handler for about box.
INT_PTR CALLBACK aboutDialog(HWND dialog_handle, UINT message, WPARAM w_param, LPARAM l_param)
{
	UNREFERENCED_PARAMETER(l_param);

	switch (message)
	{
		case WM_INITDIALOG:
		{
			return static_cast<INT_PTR>(TRUE);
		}

		case WM_COMMAND:
		{
			return static_cast<INT_PTR>(onOkCancelCommand(dialog_handle, w_param));
		}
	}

	return static_cast<INT_PTR>(FALSE);
}

// Message handler for main window.
LRESULT CALLBACK windowProcedure(HWND window_handle, UINT message, WPARAM w_param, LPARAM l_param)
{
	switch (message)
	{
		case WM_COMMAND:
		{
			// Parse the menu selections:
			switch (LOWORD(w_param))
			{
				case IDM_APP_OPEN:
				{
					PathSearchApp::getInstance()->onFileOpen(window_handle);
					break;
				}

				case IDM_APP_EXIT:
				{
					DestroyWindow(window_handle);
					break;
				}

				case IDM_APP_ABOUT:
				{
					DialogBox(PathSearchApp::getInstance()->getApplicationHandle(),
							  MAKEINTRESOURCE(IDD_ABOUTBOX), window_handle, aboutDialog);
					break;
				}

				case IDM_PATHPLANNER_RESET:
				{
					PathSearchApp::getInstance()->onReset();
					break;
				}

				case IDM_PATHPLANNER_SET_START:
				{
					PathSearchApp::getInstance()->onSetStart();
					break;
				}

				case IDM_PATHPLANNER_SET_GOAL:
				{
					PathSearchApp::getInstance()->onSetGoal();
					break;
				}

				case IDM_PATHPLANNER_RUN:
				{
					PathSearchApp::getInstance()->onRun();
					break;
				}

				case IDM_PATHPLANNER_STEP:
				{
					PathSearchApp::getInstance()->onStep();
					break;
				}

				case IDM_PATHPLANNER_TIME_RUN:
				{
					PathSearchApp::getInstance()->onTimeRun();
					break;
				}
			}

			break;
		}

		case WM_NOTIFY:
		{
			return PathSearchApp::getInstance()->onListView(l_param);
		}

		case WM_KEYDOWN:
		{
			if (PathSearchApp::getInstance()->onKeyPress(w_param))
			{
				return 0;
			}
			else
			{
				break;
			}
		}

		case WM_DESTROY:
		{
			PostQuitMessage(0);
			return 0;
		}
	}

	return DefWindowProc(window_handle, message, w_param, l_param);
}

// Message handler for tile grid window.
LRESULT CALLBACK tileGridProcedure(HWND window_handle, UINT message,
                                   WPARAM w_param, LPARAM l_param)
{
	switch (message)
	{
		case WM_KEYDOWN:
		{
			if (PathSearchApp::getInstance()->onKeyPress(w_param))
			{
				return 0;
			}
			else
			{
				break;
			}
		}

		case WM_PAINT:
		{
			PAINTSTRUCT ps;

			PathSearchApp::getInstance()->paintTileGrid(BeginPaint(window_handle, &ps));
			EndPaint(window_handle, &ps);
			return 0;
		}

		case WM_SIZE:
		{
			PathSearchApp::getInstance()->onSize(window_handle, w_param, l_param);
			return 0;
		}

		case WM_HSCROLL:
		{
			PathSearchApp::getInstance()->onHScroll(window_handle, w_param, l_param);
			return 0;
		}

		case WM_VSCROLL:
		{
			PathSearchApp::getInstance()->onVScroll(window_handle, w_param, l_param);
			return 0;
		}
	}

	return DefWindowProc(window_handle, message, w_param, l_param);
}

DWORD APIENTRY plannerThread(LPVOID unused)
{
	PathSearchApp::getInstance()->threadHandler();
	return 0;
}

CriticalSectionSynchronizer::CriticalSectionSynchronizer() : cs_()
{
	InitializeCriticalSection(&cs_);
}

CriticalSectionSynchronizer::~CriticalSectionSynchronizer()
{
	DeleteCriticalSection(&cs_);
}

void CriticalSectionSynchronizer::acquire()
{
	EnterCriticalSection(&cs_);
}

void CriticalSectionSynchronizer::release()
{
	LeaveCriticalSection(&cs_);
}

MutexSynchronizer::MutexSynchronizer() : mutex_(CreateMutex(0, FALSE, 0))
{
}

MutexSynchronizer::~MutexSynchronizer()
{
}

void MutexSynchronizer::acquire()
{
	DWORD wait_result;

	do
	{
		wait_result = WaitForSingleObject(mutex_, INFINITE);
	}
	while (!(wait_result == WAIT_OBJECT_0) && !(wait_result == WAIT_ABANDONED));
}

void MutexSynchronizer::release()
{
	ReleaseMutex(mutex_);
}

PathSearchApp *PathSearchApp::instance_ = NULL;

PathSearchApp::PathSearchApp()
	: application_handle_(0)
	, parameter_list_view_item_count_(0)
	, window_handle_(0)
	, globals_list_view_handle_(0)
	, parameter_list_view_handle_(0)
	, open_button_handle_(0)
	, reset_button_handle_(0)
	, start_waypoint_button_handle_(0)
	, goal_waypoint_button_handle_(0)
	, run_button_handle_(0)
	, step_button_handle_(0)
	, time_run_button_handle_(0)
	, tile_grid_handle_(0)
	, tile_grid_device_context_handle_(0)
	, tile_grid_buffer_context_handle_(0)
	, tile_grid_buffer_bitmap_handle_(0)
	, pause_icon_handle_(0)
	, play_icon_handle_(0)
	, tile_grid_x_(0)
	, tile_grid_y_(40)
	, tile_grid_width_(768)
	, tile_grid_height_(0)
	, tile_map_width_(0)
	, tile_map_height_(0)
	, tile_grid_offset_()
	, cannot_close_thread_handle_(true)
	, should_stop_thread_handle_(false)
	, needs_full_render_(false)
	, planner_sync_()
	, thread_sync_()
	, thread_id_(0)
	, thread_handle_(CreateThread(0, 0, plannerThread, 0, CREATE_SUSPENDED, &thread_id_))
	, ground_up_tile_map_()
	, current_planner_(new GroundUpPathSearch(ground_up_tile_map_))
{
	resetOffsets_();

	// This is a FOB's way to store a file filter string.
	text_filter_[0] = 'T';
	text_filter_[1] = 'e';
	text_filter_[2] = 'x';
	text_filter_[3] = 't';
	text_filter_[4] = '\0';
	text_filter_[5] = '*';
	text_filter_[6] = '.';
	text_filter_[7] = 'T';
	text_filter_[8] = 'X';
	text_filter_[9] = 'T';
	text_filter_[10] = '\0';
	text_filter_[11] = '\0';

	// Initialize the tooltip strings.
	_stprintf(open_button_text_, _T("%s"), _T("Open Tile Map"));
	_stprintf(reset_button_text_, _T("%s"), _T("Reset Search"));
	_stprintf(start_waypoint_button_text_, _T("%s"), _T("Set Start"));
	_stprintf(goal_waypoint_button_text_, _T("%s"), _T("Set Goal"));
	_stprintf(run_button_text_, _T("%s"), _T("Run"));
	_stprintf(step_button_text_, _T("%s"), _T("Step"));
	_stprintf(time_run_button_text_, _T("%s"), _T("Time Run"));
}

PathSearchApp::~PathSearchApp()
{
	thread_sync_.acquire();
	should_stop_thread_handle_ = true;
	thread_sync_.release();

	while (cannot_close_thread_handle_)
	{
		Sleep(100);
	}

	DeleteObject(tile_grid_buffer_bitmap_handle_);
	DeleteDC(tile_grid_buffer_context_handle_);
	ReleaseDC(window_handle_, tile_grid_device_context_handle_);
	CloseHandle(thread_handle_);
	thread_handle_ = 0;
	thread_id_ = 0;
	current_planner_->shutdownSearch();
	delete current_planner_;
}

PathSearchApp *PathSearchApp::getInstance()
{
	if (instance_ == 0)
	{
		instance_ = new PathSearchApp();
	}

	return instance_;
}

void PathSearchApp::deleteInstance()
{
	if (instance_ != 0)
	{
		delete instance_;
		instance_ = 0;
	}
}

void PathSearchApp::resetOffsets_()
{
	tile_grid_offset_.x = tile_grid_offset_.y = 0;
}

void PathSearchApp::showRunStopped_()
{
	SendMessage(run_button_handle_, BM_SETIMAGE, IMAGE_ICON,
	            reinterpret_cast<LPARAM>(play_icon_handle_));
}

void PathSearchApp::updateRunButtons_()
{
	BOOL should_enable = (
		tile_map_width_ && tile_map_height_
	) ? current_planner_->shouldEnableRun() : FALSE;

	EnableWindow(run_button_handle_, should_enable);
	EnableWindow(step_button_handle_, should_enable);
}

void PathSearchApp::updateButtons_()
{
	if (tile_map_width_ && tile_map_height_)
	{
		EnableWindow(reset_button_handle_, TRUE);
		EnableWindow(start_waypoint_button_handle_, current_planner_->shouldEnableSetStart());
		EnableWindow(goal_waypoint_button_handle_, current_planner_->shouldEnableSetGoal());
		EnableWindow(time_run_button_handle_, current_planner_->shouldEnableTimeRun());
	}
	else
	{
		EnableWindow(reset_button_handle_, FALSE);
		EnableWindow(start_waypoint_button_handle_, FALSE);
		EnableWindow(goal_waypoint_button_handle_, FALSE);
		EnableWindow(time_run_button_handle_, FALSE);
	}

	updateRunButtons_();
}

void PathSearchApp::updateParameterListView_()
{
	LVITEM list_view_item;
	int new_count = current_planner_->getInputCount();

	while (new_count < parameter_list_view_item_count_)
	{
		ListView_DeleteItem(parameter_list_view_handle_, --parameter_list_view_item_count_);
	}

	list_view_item.mask = LVIF_TEXT;
	list_view_item.pszText = LPSTR_TEXTCALLBACK;

	while (parameter_list_view_item_count_ < new_count)
	{
		list_view_item.iItem = parameter_list_view_item_count_;
		list_view_item.iSubItem = 0;
		ListView_InsertItem(parameter_list_view_handle_, &list_view_item);
		list_view_item.iSubItem = 1;
		SendMessage(parameter_list_view_handle_, LVM_SETITEM, 0,
		            reinterpret_cast<LPARAM>(&list_view_item));
		++parameter_list_view_item_count_;
	}

	InvalidateRect(parameter_list_view_handle_, 0, TRUE);
}

void PathSearchApp::updateTileGrid_()
{
	resetOffsets_();

	SCROLLINFO scroll_info;

	scroll_info.cbSize = sizeof(SCROLLINFO);
	scroll_info.fMask  = SIF_POS;
	scroll_info.nPos   = 0;
	MoveWindow(tile_grid_handle_, 0, 0, 0, 0, FALSE);
	MoveWindow(
	    tile_grid_handle_
	  , tile_grid_x_
	  , tile_grid_y_
	  , tile_grid_width_
	  , tile_grid_height_
	  , FALSE
	);
	SetScrollInfo(tile_grid_handle_, SB_HORZ, &scroll_info, TRUE);
	SetScrollInfo(tile_grid_handle_, SB_VERT, &scroll_info, TRUE);
	InvalidateRect(tile_grid_handle_, 0, FALSE);
}

void PathSearchApp::openFile_(HWND window_handle, TCHAR* file_name)
{
	LPWSTR filename;
#if defined(UNICODE) || defined(_UNICODE)
    filename = file_name;
#else
    int length = strnlen(file_name, 255) + 1;
    filename = new WCHAR[length];
    MultiByteToWideChar(CP_UTF8, 0, file_name, -1, filename, length);
#endif

#ifdef WINELIB
	basic_ifstream<TCHAR> input_file_stream(wine_get_unix_file_name(filename));
#else
	basic_ifstream<TCHAR> input_file_stream(file_name);
#endif

	if (input_file_stream.good())
	{
		planner_sync_.acquire();
		PathSearchGlobals::getInstance()->turnOff(PathSearchGlobals::SHOW_SEARCH_RUNNING);
		current_planner_->resetSearch();
		tempSolution.clear();

		bool has_read = load(input_file_stream, ground_up_tile_map_);

		if (has_read)
		{
			current_planner_->shutdownSearch();
			has_read = has_read && current_planner_->read(input_file_stream);
		}

		if (has_read)
		{
			int column_count = ground_up_tile_map_.getColumnCount();
			// VISUAL HACK.
			double h_radius = ((column_count < 20) ? 354.0 : 372.0) / column_count;
			double min_radius = PathSearchGlobals::getInstance()->getMinTileRadius();

			if (h_radius < min_radius)
			{
				h_radius = min_radius;
			}

			int row_count = ground_up_tile_map_.getRowCount();
			// VISUAL HACK.
			double v_radius = ((row_count < 20) ? 338.0 : 360.0) / row_count;

			if (v_radius < min_radius)
			{
				v_radius = min_radius;
			}

			min_radius = (h_radius < v_radius) ? h_radius : v_radius;
			ground_up_tile_map_.setRadius(min_radius);
			current_planner_->initialize();
			tile_map_width_ = static_cast<int>(((column_count << 1) | 1) * min_radius);
			tile_map_height_ = static_cast<int>((row_count * 3 + 1) * min_radius / sqrt(3.0));
		}
		else
		{

			current_planner_->resetParameters();
			ground_up_tile_map_.reset();
			tile_map_width_ = tile_map_height_ = 0;
		}

		planner_sync_.release();
		showRunStopped_();
		updateButtons_();

		if (!has_read)
		{
			MessageBox(window_handle, _T("Invalid file."), _T("IO Error"), MB_OK);
		}

		updateParameterListView_();
		updateTileGrid_();
		input_file_stream.close();
		input_file_stream.clear();
	}
}

BOOL PathSearchApp::initializeApplication(HINSTANCE application_handle, int n_cmd_show)
{
	TCHAR buffer[MAX_LOADSTRING];
	TCHAR app_class_string[MAX_LOADSTRING];

	INITCOMMONCONTROLSEX init_common_controls;

	init_common_controls.dwSize = sizeof(INITCOMMONCONTROLSEX);
	init_common_controls.dwICC = ICC_LISTVIEW_CLASSES | ICC_TAB_CLASSES;
	InitCommonControlsEx(&init_common_controls);

	LoadString(application_handle, IDS_APP_TITLE, buffer, MAX_LOADSTRING);

	WNDCLASSEX window_class_ex;

	window_class_ex.cbSize = sizeof(WNDCLASSEX);
	window_class_ex.style = 0;
	window_class_ex.lpfnWndProc = windowProcedure;
	window_class_ex.cbClsExtra = 0;
	window_class_ex.cbWndExtra = 0;
	window_class_ex.hInstance = application_handle_ = application_handle;
	window_class_ex.hIcon = LoadIcon(application_handle, MAKEINTRESOURCE(IDI_FULLSAIL_AI));
	window_class_ex.hCursor = LoadCursor(0, IDC_ARROW);
	window_class_ex.hbrBackground = reinterpret_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));
	window_class_ex.lpszMenuName = MAKEINTRESOURCE(IDC_PATHPLANNER);
	window_class_ex.lpszClassName = app_class_string;
	window_class_ex.hIconSm = LoadIcon(application_handle, MAKEINTRESOURCE(IDI_FULLSAIL_AI));
	LoadString(application_handle, IDC_PATHPLANNER, app_class_string, MAX_LOADSTRING);
	RegisterClassEx(&window_class_ex);

	window_handle_ = CreateWindow(
		app_class_string
	  , buffer
	  , (
			WS_OVERLAPPED
		  | WS_CAPTION
		  | WS_SYSMENU
		  | WS_MINIMIZEBOX
		)
	  , CW_USEDEFAULT
	  , CW_USEDEFAULT
	  , 1024
	  , 768
	  , 0
	  , 0
	  , application_handle
	  , 0
	);

	if (window_handle_ != NULL)
	{
		LONG const y_offset_1 = 0;
		RECT client_rectangle;

		GetClientRect(window_handle_, &client_rectangle);
		client_rectangle.right -= client_rectangle.left;
		client_rectangle.bottom -= client_rectangle.top;
		client_rectangle.left = client_rectangle.top = 0;
		tile_grid_height_ = client_rectangle.bottom - tile_grid_y_;

		LONG const height_0 = tile_grid_y_ - y_offset_1;
		HICON icon_handle;

		open_button_handle_ = CreateWindow(
			_T("BUTTON")
		  , open_button_text_
		  , WS_TABSTOP | WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | BS_ICON
		  , 0
		  , y_offset_1
		  , height_0
		  , height_0
		  , window_handle_
		  , reinterpret_cast<HMENU>(IDM_APP_OPEN)
		  , application_handle
		  , 0
		);
		if (open_button_handle_ != NULL)
		{
			icon_handle = LoadIcon(application_handle, MAKEINTRESOURCE(IDI_OPEN));
			SendMessage(open_button_handle_, BM_SETIMAGE, IMAGE_ICON,
			            reinterpret_cast<LPARAM>(icon_handle));
			EnableWindow(open_button_handle_, TRUE);
		}
		else
		{
			MessageBox(window_handle_, _T("Creation Failed"), _T("open_button_handle_"), MB_OK);
			DestroyWindow(window_handle_);
			return FALSE;
		}

		reset_button_handle_ = CreateWindow(
			_T("BUTTON")
		  , reset_button_text_
		  , WS_TABSTOP | WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | BS_ICON
		  , height_0
		  , y_offset_1
		  , height_0
		  , height_0
		  , window_handle_
		  , reinterpret_cast<HMENU>(IDM_PATHPLANNER_RESET)
		  , application_handle
		  , 0
		);
		if (reset_button_handle_ != NULL)
		{
			icon_handle = LoadIcon(application_handle, MAKEINTRESOURCE(IDI_STOP));
			SendMessage(reset_button_handle_, BM_SETIMAGE, IMAGE_ICON,
			            reinterpret_cast<LPARAM>(icon_handle));
			EnableWindow(reset_button_handle_, TRUE);
		}
		else
		{
			MessageBox(window_handle_, _T("Creation Failed"), _T("reset_button_handle_"), MB_OK);
			DestroyWindow(window_handle_);
			return FALSE;
		}

		start_waypoint_button_handle_ = CreateWindow(
			_T("BUTTON")
		  , start_waypoint_button_text_
		  , WS_TABSTOP | WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | BS_ICON
		  , height_0 << 1
		  , y_offset_1
		  , height_0
		  , height_0
		  , window_handle_
		  , reinterpret_cast<HMENU>(IDM_PATHPLANNER_SET_START)
		  , application_handle
		  , 0
		);
		if (start_waypoint_button_handle_ != NULL)
		{
			icon_handle = LoadIcon(application_handle, MAKEINTRESOURCE(IDI_START));
			SendMessage(start_waypoint_button_handle_, BM_SETIMAGE, IMAGE_ICON,
			            reinterpret_cast<LPARAM>(icon_handle));
			EnableWindow(start_waypoint_button_handle_, current_planner_->shouldEnableSetStart());
		}
		else
		{
			MessageBox(window_handle_, _T("Creation Failed"), _T("start_waypoint_button_handle_"),
			           MB_OK);
			DestroyWindow(window_handle_);
			return FALSE;
		}

		goal_waypoint_button_handle_ = CreateWindow(
			_T("BUTTON")
		  , goal_waypoint_button_text_
		  , WS_TABSTOP | WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | BS_ICON
		  , height_0 * 3
		  , y_offset_1
		  , height_0
		  , height_0
		  , window_handle_
		  , reinterpret_cast<HMENU>(IDM_PATHPLANNER_SET_GOAL)
		  , application_handle
		  , 0
		);
		if (goal_waypoint_button_handle_ != NULL)
		{
			icon_handle = LoadIcon(application_handle, MAKEINTRESOURCE(IDI_GOAL));
			SendMessage(goal_waypoint_button_handle_, BM_SETIMAGE, IMAGE_ICON,
			            reinterpret_cast<LPARAM>(icon_handle));
			EnableWindow(goal_waypoint_button_handle_, current_planner_->shouldEnableSetGoal());
		}
		else
		{
			MessageBox(window_handle_, _T("Creation Failed"), _T("goal_waypoint_button_handle_"),
			           MB_OK);
			DestroyWindow(window_handle_);
			return FALSE;
		}

		run_button_handle_ = CreateWindow(
			_T("BUTTON")
		  , run_button_text_
		  , WS_TABSTOP | WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | BS_ICON
		  , height_0 << 2
		  , y_offset_1
		  , height_0
		  , height_0
		  , window_handle_
		  , reinterpret_cast<HMENU>(IDM_PATHPLANNER_RUN)
		  , application_handle
		  , 0
		);
		if (run_button_handle_ != NULL)
		{
			pause_icon_handle_ = LoadIcon(application_handle, MAKEINTRESOURCE(IDI_PAUSE));
			play_icon_handle_ = LoadIcon(application_handle, MAKEINTRESOURCE(IDI_PLAY));
			SendMessage(run_button_handle_, BM_SETIMAGE, IMAGE_ICON,
			            reinterpret_cast<LPARAM>(play_icon_handle_));
			EnableWindow(run_button_handle_, current_planner_->shouldEnableRun());
		}
		else
		{
			MessageBox(window_handle_, _T("Creation Failed"), _T("run_button_handle_"), MB_OK);
			DestroyWindow(window_handle_);
			return FALSE;
		}

		step_button_handle_ = CreateWindow(
			_T("BUTTON")
		  , step_button_text_
		  , WS_TABSTOP | WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | BS_ICON
		  , height_0 * 5
		  , y_offset_1
		  , height_0
		  , height_0
		  , window_handle_
		  , reinterpret_cast<HMENU>(IDM_PATHPLANNER_STEP)
		  , application_handle
		  , 0
		);
		if (step_button_handle_ != NULL)
		{
			icon_handle = LoadIcon(application_handle, MAKEINTRESOURCE(IDI_PLAY_PAUSE));
			SendMessage(step_button_handle_, BM_SETIMAGE, IMAGE_ICON,
			            reinterpret_cast<LPARAM>(icon_handle));
			EnableWindow(step_button_handle_, current_planner_->shouldEnableRun());
		}
		else
		{
			MessageBox(window_handle_, _T("Creation Failed"), _T("step_button_handle_"), MB_OK);
			DestroyWindow(window_handle_);
			return FALSE;
		}

		time_run_button_handle_ = CreateWindow(
			_T("BUTTON")
		  , time_run_button_text_
		  , WS_TABSTOP | WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | BS_ICON
		  , height_0 * 6
		  , y_offset_1
		  , height_0
		  , height_0
		  , window_handle_
		  , reinterpret_cast<HMENU>(IDM_PATHPLANNER_TIME_RUN)
		  , application_handle
		  , 0
		);
		if (time_run_button_handle_ != NULL)
		{
			icon_handle = LoadIcon(application_handle, MAKEINTRESOURCE(IDI_PLAY_END));
			SendMessage(time_run_button_handle_, BM_SETIMAGE, IMAGE_ICON,
			            reinterpret_cast<LPARAM>(icon_handle));
			EnableWindow(time_run_button_handle_, current_planner_->shouldEnableTimeRun());
		}
		else
		{
			MessageBox(window_handle_, _T("Creation Failed"), _T("time_run_button_handle_"),
			           MB_OK);
			DestroyWindow(window_handle_);
			return FALSE;
		}

		LONG const x_offset = 770;
		LONG const y_offset_2 = 24;
		LONG const width = client_rectangle.right - 772;
		LONG const height_1 = 32;

		LoadString(application_handle, IDS_LV_GLOBALS, buffer, MAX_LOADSTRING);

		globals_list_view_handle_ = CreateWindow(
			WC_LISTVIEW
		  , buffer
		  , WS_CHILD | WS_VISIBLE | LVS_REPORT | LVS_EDITLABELS
		  , x_offset
		  , y_offset_2 << 1
		  , width
		  , height_1
		  , window_handle_
		  , 0
		  , application_handle
		  , 0
		);
		if (globals_list_view_handle_ != NULL)
		{
			EnableWindow(globals_list_view_handle_, FALSE);
		}
		else
		{
			MessageBox(window_handle_, _T("Creation Failed"), _T("globals_list_view_handle_"),
			           MB_OK);
			DestroyWindow(window_handle_);
			return FALSE;
		}

		LONG const y_offset_4 = (y_offset_2 << 1) + height_1;
		LONG const height_2 = 128;

		LoadString(application_handle, IDS_LV_PARAMETERS, buffer, MAX_LOADSTRING);

		if (!(
			parameter_list_view_handle_ = CreateWindow(
			    WC_LISTVIEW
			  , buffer
			  , WS_CHILD | WS_VISIBLE | LVS_REPORT | LVS_EDITLABELS
			  , x_offset
			  , y_offset_4
			  , width
			  , height_2
			  , window_handle_
			  , 0
			  , application_handle
			  , 0
			)
		))
		{
			MessageBox(window_handle_, _T("Creation Failed"), _T("parameter_list_view_handle_"),
			           MB_OK);
			DestroyWindow(window_handle_);
			return FALSE;
		}

		window_class_ex.lpfnWndProc = tileGridProcedure;
		LoadString(application_handle, IDC_TILEGRID, app_class_string, MAX_LOADSTRING);
		RegisterClassEx(&window_class_ex);

		tile_grid_handle_ = CreateWindow(
			app_class_string
		  , 0
		  , WS_CHILD | WS_VISIBLE
		  , tile_grid_x_
		  , tile_grid_y_
		  , tile_grid_width_
		  , tile_grid_height_
		  , window_handle_
		  , 0
		  , application_handle
		  , 0
		);
		if (tile_grid_handle_ == NULL)
		{
			MessageBox(window_handle_, _T("Creation Failed"), _T("tile_grid_handle_"), MB_OK);
			DestroyWindow(window_handle_);
			return FALSE;
		}
/*
		if (icon_handle)
		{
			HWND tooltip_handle;
			TOOLINFO tool_info = { 0 };

			tool_info.cbSize = sizeof(tool_info);
			tool_info.hwnd = window_handle_;
			tool_info.hinst = application_handle;
			tool_info.uFlags = TTF_IDISHWND | TTF_SUBCLASS | TTF_TRANSPARENT;

			if (
				tooltip_handle = CreateWindowEx(
				    WS_EX_TOPMOST
				  , TOOLTIPS_CLASS
				  , 0
				  , (
				        WS_POPUP
				      | TTS_NOPREFIX
				      | TTS_ALWAYSTIP
				      | TTS_BALLOON
				    )
				  , CW_USEDEFAULT
				  , CW_USEDEFAULT
				  , CW_USEDEFAULT
				  , CW_USEDEFAULT
				  , window_handle_
				  , 0
				  , application_handle
				  , 0
				)
			)
			{
				SetWindowPos(
				    tooltip_handle
				  , HWND_TOPMOST
				  , 0
				  , 0
				  , 0
				  , 0
				  , SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE
				);
//				tool_info.uId = GetWindowLong(open_button_handle_, GWL_ID)
				tool_info.uId = reinterpret_cast<UINT_PTR>(
					open_button_handle_
				);
				tool_info.lpszText = open_button_text_;
				SendMessage(tooltip_handle, TTM_ADDTOOL, 0, reinterpret_cast<LPARAM>(&tool_info));
				SendMessage(tooltip_handle, TTM_ACTIVATE, TRUE, 0);
			}
		}
*/

		LVCOLUMN lv_column;

		lv_column.mask = LVCF_FMT | LVCF_WIDTH | LVCF_TEXT | LVCF_SUBITEM;
		lv_column.pszText = buffer;
		lv_column.fmt = LVCFMT_RIGHT;
		lv_column.cx = 80;
		LoadString(application_handle, IDS_LVC_VALUE, buffer, MAX_LOADSTRING);
		ListView_InsertColumn(globals_list_view_handle_, lv_column.iSubItem = 0, &lv_column);
		ListView_InsertColumn(parameter_list_view_handle_, lv_column.iSubItem, &lv_column);
		lv_column.fmt = LVCFMT_LEFT;
		lv_column.cx = 144;
		LoadString(application_handle, IDS_LVC_GLOBAL_SETTING, buffer, MAX_LOADSTRING);
		ListView_InsertColumn(globals_list_view_handle_, lv_column.iSubItem = 1, &lv_column);
		LoadString(application_handle, IDS_LVC_PARAM_NAME, buffer, MAX_LOADSTRING);
		ListView_InsertColumn(parameter_list_view_handle_, lv_column.iSubItem, &lv_column);

		LVITEM list_view_item;

		list_view_item.mask = LVIF_TEXT;
		list_view_item.pszText = LPSTR_TEXTCALLBACK;
		list_view_item.iItem = 0;
		list_view_item.iSubItem = 0;
		ListView_InsertItem(globals_list_view_handle_, &list_view_item);
		list_view_item.iSubItem = 1;
		SendMessage(globals_list_view_handle_, LVM_SETITEM, 0,
		            reinterpret_cast<LPARAM>(&list_view_item));

		tile_grid_device_context_handle_ = GetDC(tile_grid_handle_);
		tile_grid_buffer_context_handle_ = CreateCompatibleDC(tile_grid_device_context_handle_);
		tile_grid_buffer_bitmap_handle_ = CreateCompatibleBitmap(tile_grid_device_context_handle_,
		                                                         tile_grid_width_,
		                                                         tile_grid_height_);
		_stprintf(buffer, _T("%s"), _T(USEDEFAULTMAP));
		openFile_(window_handle_, buffer);
		ShowWindow(window_handle_, n_cmd_show);
		ResumeThread(thread_handle_);
		UpdateWindow(window_handle_);

		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

void PathSearchApp::renderFull_()
{
	BitBlt(
	    tile_grid_buffer_context_handle_
	  , 0
	  , 0
	  , tile_grid_width_
	  , tile_grid_height_
	  , tile_grid_buffer_context_handle_
	  , 0
	  , 0
	  , BLACKNESS
	);
	paintAllTileGrid_(tile_grid_device_context_handle_, tile_grid_buffer_context_handle_);
	needs_full_render_ = current_planner_->needsFullRedraw();
}

void PathSearchApp::threadHandler()
{
	HGDIOBJ old_tile_grid_bitmap_handle = SelectObject(tile_grid_buffer_context_handle_,
	                                                   tile_grid_buffer_bitmap_handle_);

	for (;;)
	{
		thread_sync_.acquire();

		if (should_stop_thread_handle_)
		{
			SelectObject(tile_grid_buffer_context_handle_, old_tile_grid_bitmap_handle);
			cannot_close_thread_handle_ = false;
			thread_sync_.release();
			return;
		}

		thread_sync_.release();
		planner_sync_.acquire();

		if (PathSearchGlobals::getInstance()->isFlagOn(PathSearchGlobals::SHOW_SEARCH_RUNNING))
		{
			if (current_planner_->isRunnableByKey())
			{
				if (needs_full_render_)
				{
					current_planner_->runSearch(/*10*/);//run search for a certain timestep
					renderFull_();
				}
				else
				{
					current_planner_->beginRedrawSearchProgress(tile_grid_offset_,
					                                            tile_grid_width_,
					                                            tile_grid_height_,
					                                            tile_grid_buffer_context_handle_);
					current_planner_->runSearch(/*10*/);
					current_planner_->endRedrawSearchProgress(tile_grid_offset_,
					                                          tile_grid_width_,
					                                          tile_grid_height_,
					                                          tile_grid_buffer_context_handle_);
					doubleBufferTileGrid_(tile_grid_device_context_handle_,
					                      tile_grid_buffer_context_handle_);
				}

				planner_sync_.release();
			}
			else if (current_planner_->isInitializableByKey())
			{
				current_planner_->setTimedSearch(false);
				current_planner_->initializeSearch();
				current_planner_->checkSolution(window_handle_);
				renderFull_();
				planner_sync_.release();
			}
			else
			{
				PathSearchGlobals::getInstance()->turnOff(PathSearchGlobals::SHOW_SEARCH_RUNNING);
				showRunStopped_();
				updateRunButtons_();
				current_planner_->checkSolution(window_handle_);
				planner_sync_.release();
				InvalidateRect(tile_grid_handle_, 0, FALSE);
			}
		}
		else
		{
			planner_sync_.release();
		}

		Sleep(25);
	}
}

void PathSearchApp::reset_()
{
	PathSearchGlobals::getInstance()->turnOff(PathSearchGlobals::SHOW_SEARCH_RUNNING);
	current_planner_->resetSearch();
	showRunStopped_();
	updateRunButtons_();
}

void PathSearchApp::onReset()
{
	planner_sync_.acquire();
	reset_();
	planner_sync_.release();
	InvalidateRect(tile_grid_handle_, 0, FALSE);
}

void PathSearchApp::resetByKey_()
{
	planner_sync_.acquire();

	if (current_planner_->isReady())
	{
		reset_();
		planner_sync_.release();
		InvalidateRect(tile_grid_handle_, 0, FALSE);
	}
	else
	{
		planner_sync_.release();
	}
}

void PathSearchApp::onSetStart()
{
	planner_sync_.acquire();

	if (current_planner_->updateStart())
	{
		planner_sync_.release();
		InvalidateRect(tile_grid_handle_, 0, FALSE);
	}
	else
	{
		planner_sync_.release();
	}
}

void PathSearchApp::onSetGoal()
{
	planner_sync_.acquire();

	if (current_planner_->updateGoal())
	{
		planner_sync_.release();
		InvalidateRect(tile_grid_handle_, 0, FALSE);
	}
	else
	{
		planner_sync_.release();
	}
}

bool PathSearchApp::run_()
{
	if (PathSearchGlobals::getInstance()->isFlagOn(PathSearchGlobals::SHOW_SEARCH_RUNNING))
	{
		PathSearchGlobals::getInstance()->turnOff(PathSearchGlobals::SHOW_SEARCH_RUNNING);
		showRunStopped_();
		updateRunButtons_();
		current_planner_->checkSolution(window_handle_);
		return true;
	}
	else
	{
		BitBlt(
		    tile_grid_buffer_context_handle_
		  , 0
		  , 0
		  , tile_grid_width_
		  , tile_grid_height_
		  , tile_grid_buffer_context_handle_
		  , 0
		  , 0
		  , BLACKNESS
		);
		PathSearchGlobals::getInstance()->turnOn(PathSearchGlobals::SHOW_SEARCH_RUNNING);
		paintAllTileGrid_(
		    tile_grid_device_context_handle_
		  , tile_grid_buffer_context_handle_
		);
		return false;
	}
}

void PathSearchApp::onRun()
{
	planner_sync_.acquire();

	if (run_())
	{
		planner_sync_.release();
		InvalidateRect(tile_grid_handle_, 0, FALSE);
	}
	else
	{
		planner_sync_.release();
		SendMessage(
		    run_button_handle_
		  , BM_SETIMAGE
		  , IMAGE_ICON
		  , reinterpret_cast<LPARAM>(pause_icon_handle_)
		);
	}
}

void PathSearchApp::runByKey_()
{
	planner_sync_.acquire();

	if (current_planner_->isReady())
	{
		if (current_planner_->isRunnableByKey() || current_planner_->isInitializableByKey())
		{
			if (run_())
			{
				planner_sync_.release();
				InvalidateRect(tile_grid_handle_, 0, FALSE);
			}
			else
			{
				planner_sync_.release();
				SendMessage(
				    run_button_handle_
				  , BM_SETIMAGE
				  , IMAGE_ICON
				  , reinterpret_cast<LPARAM>(pause_icon_handle_)
				);
			}

			return;
		}
	}

	planner_sync_.release();
}

void PathSearchApp::step_()
{
	PathSearchGlobals::getInstance()->turnOff(PathSearchGlobals::SHOW_SEARCH_RUNNING);

	if (current_planner_->isInitializableByKey())
	{
		current_planner_->setTimedSearch(false);
		//current_planner_->resetSearch();
		current_planner_->initializeSearch();
		current_planner_->stepSearch(/*0*/);//one step at a time
	}
	else if (current_planner_->isRunnableByKey())
	{
		current_planner_->stepSearch(/*0*/);//one step at a time
	}

	showRunStopped_();
	updateRunButtons_();
	current_planner_->checkSolution(window_handle_);
}

void PathSearchApp::onStep()
{
	planner_sync_.acquire();
	step_();
	planner_sync_.release();
	InvalidateRect(tile_grid_handle_, 0, FALSE);
}

void PathSearchApp::stepByKey_()
{
	planner_sync_.acquire();

	if (current_planner_->isReady())
	{
		step_();
		planner_sync_.release();
		InvalidateRect(tile_grid_handle_, 0, FALSE);
	}
	else
	{
		planner_sync_.release();
	}
}

void PathSearchApp::timeRun_()
{
	PathSearchGlobals::getInstance()->turnOff(PathSearchGlobals::SHOW_SEARCH_RUNNING);
	showRunStopped_();
	EnableWindow(start_waypoint_button_handle_, FALSE);
	EnableWindow(goal_waypoint_button_handle_, FALSE);
	EnableWindow(run_button_handle_, FALSE);
	EnableWindow(step_button_handle_, FALSE);
	EnableWindow(time_run_button_handle_, FALSE);
	current_planner_->resetSearch();
	current_planner_->timeSearch();
	updateButtons_();
	//current_planner_->checkSolution(window_handle_);//we can't test the solution on the timed run
}

void PathSearchApp::onTimeRun()
{
	planner_sync_.acquire();
	timeRun_();
	planner_sync_.release();
	InvalidateRect(tile_grid_handle_, 0, FALSE);
}

void PathSearchApp::timeRunByKey_()
{
	planner_sync_.acquire();

	if (current_planner_->isReady())
	{
		timeRun_();
		planner_sync_.release();
		InvalidateRect(tile_grid_handle_, 0, FALSE);
	}
	else
	{
		planner_sync_.release();
	}
}

bool PathSearchApp::onKeyPress(WPARAM w_param)
{
	if (tile_map_width_ && tile_map_height_)
	{
		switch (w_param)
		{
			case VK_BACK:
			{
				resetByKey_();
				return true;
			}

			case VK_SPACE:
			{
				runByKey_();
				return true;
			}

			case VK_ADD:
			{
				stepByKey_();
				return true;
			}

			case VK_TAB:
			{
				timeRunByKey_();
				return true;
			}

			case 0x53:  // 'S'
			{
				onSetStart();
				return true;
			}

			case 0x47:  // 'G'
			{
				onSetGoal();
				return true;
			}
		}
	}

	return false;
}

LRESULT PathSearchApp::onListView(LPARAM l_param)
{
	NMHDR* p_nmhdr = reinterpret_cast<NMHDR*>(l_param);

	switch (p_nmhdr->code)
	{
		case LVN_KEYDOWN:
		{
			onKeyPress(reinterpret_cast<NMLVKEYDOWN*>(l_param)->wVKey);
			break;
		}

		case NM_CLICK:
		{
			NMITEMACTIVATE* p_nm_item = reinterpret_cast<NMITEMACTIVATE*>(l_param);
			HWND list_view_handle = p_nmhdr->hwndFrom;
			int row = p_nm_item->iItem;

			// Ensure that a list-view item was selected.
			if (-1 != row)
			{
				// Don't take any chances with editable list-views.
				planner_sync_.acquire();
				PathSearchGlobals::getInstance()->turnOff(
					PathSearchGlobals::SHOW_SEARCH_RUNNING
				);
				showRunStopped_();
				updateRunButtons_();
				planner_sync_.release();
				ListView_EditLabel(list_view_handle, row);
			}
		}

		case LVN_ITEMACTIVATE:
		{
			NMITEMACTIVATE* p_nm_item = reinterpret_cast<NMITEMACTIVATE*>(l_param);
			int row = p_nm_item->iItem;

			// Ensure that a list-view item was selected.
			if (-1 != row)
			{
				HWND list_view_handle = p_nmhdr->hwndFrom;

				// Don't take any chances with editable list-views.
				planner_sync_.acquire();
				PathSearchGlobals::getInstance()->turnOff(
					PathSearchGlobals::SHOW_SEARCH_RUNNING
				);
				showRunStopped_();
				updateRunButtons_();
				planner_sync_.release();
				ListView_EditLabel(list_view_handle, row);
			}

			break;
		}

		case LVN_ENDLABELEDIT:
		{
			NMLVDISPINFO* list_view_display_info = reinterpret_cast<NMLVDISPINFO*>(l_param);
			LVITEM& item = list_view_display_info->item;

			// Ensure that the user has not cancelled label editing.
			if (item.pszText)
			{
				if (p_nmhdr->hwndFrom == parameter_list_view_handle_)
				{
					planner_sync_.acquire();

					if (current_planner_->updateInput(list_view_display_info))
					{
						planner_sync_.release();
						InvalidateRect(tile_grid_handle_, 0, FALSE);
						return TRUE;
					}
					else
					{
						planner_sync_.release();
					}
				}
				else// if (p_nmhdr->hwndFrom == globals_list_view_handle_)
				{
					switch (item.iItem)
					{
						case 0:
						{
							double radius = stof(item.pszText);

							if (radius < 4.0)
							{
								radius = 4.0;
							}

							PathSearchGlobals::getInstance()->setMinTileRadius(radius);
							return TRUE;
						}
					}
				}
			}

			return FALSE;
		}

		case NM_CUSTOMDRAW:
		{
			return CDRF_DODEFAULT;
		}

		case LVN_GETDISPINFO:
		{
			NMLVDISPINFO* list_view_display_info = reinterpret_cast<NMLVDISPINFO*>(l_param);
			HWND list_view_handle = p_nmhdr->hwndFrom;

			if (list_view_handle == parameter_list_view_handle_)
			{
				planner_sync_.acquire();
				current_planner_->displayInput(list_view_display_info);
				planner_sync_.release();
			}
			else// if (list_view_handle == globals_list_view_handle_)
			{
				LVITEM& item = list_view_display_info->item;

				switch (item.iItem)
				{
					case 0:
					{
						if (item.iSubItem)
						{
							_stprintf(
							    item.pszText
							  , _T("%s")
							  , _T("Minimum Tile Radius")
							);
						}
						else
						{
							_stprintf(
							    item.pszText
							  , _T("%1.6f")
							  , PathSearchGlobals::getInstance()->getMinTileRadius()
							);
						}

						break;
					}
				}
			}

			break;
		}
	}

	return FALSE;
}

void PathSearchApp::onSize(HWND window_handle, WPARAM w_param, LPARAM l_param)
{
	SCROLLINFO horizontal_scroll_info;
	SCROLLINFO vertical_scroll_info;

	// Get the client dimensions and set the scrollbar properties.
	horizontal_scroll_info.cbSize = vertical_scroll_info.cbSize = sizeof(SCROLLINFO);
	horizontal_scroll_info.fMask = vertical_scroll_info.fMask = (
	    SIF_RANGE
	  | SIF_PAGE
	  | SIF_DISABLENOSCROLL
	);
	horizontal_scroll_info.nMin = vertical_scroll_info.nMin = 0;
	horizontal_scroll_info.nPage = LOWORD(l_param);
	vertical_scroll_info.nPage = HIWORD(l_param);
	horizontal_scroll_info.nMax = tile_map_width_ - 1;
	vertical_scroll_info.nMax = tile_map_height_ - 1;

	// Enable the horizontal scroll bar.
	SetScrollInfo(window_handle, SB_HORZ, &horizontal_scroll_info, TRUE);

	// Enable the vertical scroll bar.
	SetScrollInfo(window_handle, SB_VERT, &vertical_scroll_info, TRUE);
}

void PathSearchApp::onHScroll(HWND window_handle, WPARAM w_param, LPARAM l_param)
{
	SCROLLINFO horizontal_scroll_info;

	// Get all information pertaining to the horizontal scroll bar.
	horizontal_scroll_info.cbSize = sizeof(SCROLLINFO);
	horizontal_scroll_info.fMask  = SIF_ALL;
	GetScrollInfo(window_handle, SB_HORZ, &horizontal_scroll_info);

	// Save the old position for later comparison.
	int const old_position = horizontal_scroll_info.nPos;

	// Adjust the scroll position based upon the scroll request.
	switch (LOWORD(w_param))
	{
		case SB_LEFT:
			horizontal_scroll_info.nPos = horizontal_scroll_info.nMin;
			break;
		case SB_RIGHT:
			horizontal_scroll_info.nPos = horizontal_scroll_info.nMax;
			break;
		case SB_LINELEFT:
			horizontal_scroll_info.nPos -= 1;
			break;
		case SB_LINERIGHT:
			horizontal_scroll_info.nPos += 1;
			break;
		case SB_PAGELEFT:
			horizontal_scroll_info.nPos -= horizontal_scroll_info.nPage;
			break;
		case SB_PAGERIGHT:
			horizontal_scroll_info.nPos += horizontal_scroll_info.nPage;
			break;
		case SB_THUMBTRACK:
			horizontal_scroll_info.nPos = horizontal_scroll_info.nTrackPos;
			break;
	}

	// Set the new, horizontal scroll position.
	horizontal_scroll_info.fMask = SIF_POS;
	SetScrollInfo(window_handle, SB_HORZ, &horizontal_scroll_info, TRUE);
	GetScrollInfo(window_handle, SB_HORZ, &horizontal_scroll_info);

	if (old_position != horizontal_scroll_info.nPos)
	{
		planner_sync_.acquire();
		tile_grid_offset_.x = -horizontal_scroll_info.nPos;

		if (PathSearchGlobals::getInstance()->isFlagOn(PathSearchGlobals::SHOW_SEARCH_RUNNING))
		{
			needs_full_render_ = true;
			planner_sync_.release();
		}
		else
		{
			planner_sync_.release();
			InvalidateRect(tile_grid_handle_, 0, FALSE);
		}
	}
}

void PathSearchApp::onVScroll(HWND window_handle, WPARAM w_param, LPARAM l_param)
{
	SCROLLINFO vertical_scroll_info;

	// Get all information pertaining to the vertical scroll bar.
	vertical_scroll_info.cbSize = sizeof(SCROLLINFO);
	vertical_scroll_info.fMask  = SIF_ALL;
	GetScrollInfo(window_handle, SB_VERT, &vertical_scroll_info);

	// Save the old position for later comparison.
	int const old_position = vertical_scroll_info.nPos;

	// Adjust the scroll position based upon the scroll request.
	switch (LOWORD(w_param))
	{
		case SB_LEFT:
			vertical_scroll_info.nPos = vertical_scroll_info.nMin;
			break;
		case SB_RIGHT:
			vertical_scroll_info.nPos = vertical_scroll_info.nMax;
			break;
		case SB_LINELEFT:
			vertical_scroll_info.nPos -= 1;
			break;
		case SB_LINERIGHT:
			vertical_scroll_info.nPos += 1;
			break;
		case SB_PAGELEFT:
			vertical_scroll_info.nPos -= vertical_scroll_info.nPage;
			break;
		case SB_PAGERIGHT:
			vertical_scroll_info.nPos += vertical_scroll_info.nPage;
			break;
		case SB_THUMBTRACK:
			vertical_scroll_info.nPos = vertical_scroll_info.nTrackPos;
			break;
	}

	// Set the new, vertical scroll position.
	vertical_scroll_info.fMask = SIF_POS;
	SetScrollInfo(window_handle, SB_VERT, &vertical_scroll_info, TRUE);
	GetScrollInfo(window_handle, SB_VERT, &vertical_scroll_info);

	if (old_position != vertical_scroll_info.nPos)
	{
		planner_sync_.acquire();
		tile_grid_offset_.y = -vertical_scroll_info.nPos;

		if (PathSearchGlobals::getInstance()->isFlagOn(PathSearchGlobals::SHOW_SEARCH_RUNNING))
		{
			needs_full_render_ = true;
			planner_sync_.release();
		}
		else
		{
			planner_sync_.release();
			InvalidateRect(tile_grid_handle_, 0, FALSE);
		}
	}
}

void PathSearchApp::onFileOpen(HWND window_handle)
{
	OPENFILENAME open_file_name;
	TCHAR        file_name[MAX_BUFFER_SIZE];
	TCHAR        initial_directory[MAX_BUFFER_SIZE];

	ZeroMemory(&open_file_name, sizeof(open_file_name));
	open_file_name.lStructSize = sizeof(open_file_name);
	open_file_name.hwndOwner = window_handle;
	open_file_name.lpstrFile = file_name;

	// Set lpstrFile[0] to '\0' so that GetOpenFileName does not 
	// use the contents of file_name to initialize itself.
	open_file_name.lpstrFile[0] = '\0';
	open_file_name.nMaxFile = sizeof(file_name);
	open_file_name.lpstrFilter = text_filter_;
	open_file_name.nFilterIndex = 1;
	open_file_name.lpstrFileTitle = 0;
	open_file_name.nMaxFileTitle = 0;
	open_file_name.lpstrInitialDir = initial_directory;
	_stprintf(initial_directory, _T("%s"), _T("./Data"));
	open_file_name.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	if (GetOpenFileName(&open_file_name))
	{
		openFile_(window_handle, file_name);
	}
}

void PathSearchApp::doubleBufferTileGrid_(HDC device_context_handle,
											   HDC buffer_context_handle) const
{
	BitBlt(
	    device_context_handle
	  , 0
	  , 0
	  , tile_grid_width_
	  , tile_grid_height_
	  , buffer_context_handle
	  , 0
	  , 0
	  , SRCCOPY
	);
}

void PathSearchApp::paintAllTileGrid_(HDC device_context_handle, HDC buffer_context_handle) const
{
	drawGrid(
	    ground_up_tile_map_
	  , tile_grid_offset_
	  , tile_grid_width_
	  , tile_grid_height_
	  , buffer_context_handle
	);
	current_planner_->displaySearchProgress(
	    tile_grid_offset_
	  , tile_grid_width_
	  , tile_grid_height_
	  , buffer_context_handle
	);
	doubleBufferTileGrid_(device_context_handle, buffer_context_handle);
}

void PathSearchApp::paintTileGrid(HDC device_context_handle) const
{
	HDC buffer_context_handle = CreateCompatibleDC(device_context_handle);
	HBITMAP buffer_bitmap_handle = CreateCompatibleBitmap(
	    device_context_handle
	  , tile_grid_width_
	  , tile_grid_height_
	);
	HGDIOBJ old_bitmap_handle = SelectObject(
	    buffer_context_handle
	  , buffer_bitmap_handle
	);

	planner_sync_.acquire();
	paintAllTileGrid_(device_context_handle, buffer_context_handle);
	planner_sync_.release();
	SelectObject(buffer_context_handle, old_bitmap_handle);
	DeleteObject(buffer_bitmap_handle);
	DeleteDC(buffer_context_handle);
}

int APIENTRY _tWinMain(HINSTANCE application_handle,
                       HINSTANCE previous_application_handle,
                       LPTSTR    command_line,
                       int       n_cmd_show)
{
	UNREFERENCED_PARAMETER(previous_application_handle);
	UNREFERENCED_PARAMETER(command_line);

//#ifdef _DEBUG
	// Set up the console window.
	CONSOLE_SCREEN_BUFFER_INFO coninfo;

	AllocConsole();
	GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &coninfo);
	coninfo.dwSize.Y = 500;
	SetConsoleScreenBufferSize(GetStdHandle(STD_OUTPUT_HANDLE), coninfo.dwSize);

#ifdef _MSC_VER
	// Redirect standard output to the console window.
	HANDLE standard_handle = GetStdHandle(STD_OUTPUT_HANDLE);
	int console_handle = _open_osfhandle(reinterpret_cast<intptr_t>(standard_handle), _O_TEXT);
	*stdout = *_fdopen(console_handle, "w");
	setvbuf(stdout, 0, _IONBF, 0);

	// Redirect standard error to the console window.
	standard_handle = GetStdHandle(STD_ERROR_HANDLE);
	console_handle = _open_osfhandle(reinterpret_cast<intptr_t>(standard_handle), _O_TEXT);
	*stderr = *_fdopen(console_handle, "w");
	setvbuf(stderr, 0, _IONBF, 0);
#endif
	// Allow C++ code to benefit from console redirection.
	ios::sync_with_stdio();
//#endif

	// Perform application initialization:
	if (!PathSearchApp::getInstance()->initializeApplication(application_handle, n_cmd_show))
	{
		PathSearchApp::deleteInstance();
		PathSearchGlobals::deleteInstance();
		return FALSE;
	}

	MSG msg;
	HACCEL accelerator_table_handle = LoadAccelerators(
	    application_handle
	  , MAKEINTRESOURCE(IDC_PATHPLANNER)
	);

	// Main message loop:
	while (GetMessage(&msg, 0, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, accelerator_table_handle, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	PathSearchApp::deleteInstance();
	PathSearchGlobals::deleteInstance();
	return static_cast<int>(msg.wParam);
}

