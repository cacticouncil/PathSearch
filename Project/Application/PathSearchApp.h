// Written 2010, Cromwell D. Enage
// Updated 2016, Jeremiah Blanchard
#pragma once

#include "../platform.h"

#include <stdlib.h>
#include <memory.h>
#include <tchar.h>

#include <fstream>
#include <utility>
#include <vector>

#include <windows.h>
#include <commdlg.h>
#include <commctrl.h>

#include "../SearchLibrary/PathSearch.h"
#include "../Resource/Resource.h"

// change this to start the program on whatever default map as you like from the table below
#define USEDEFAULTMAP hex035x035

#define hex006x006 "./Data/hex006x006.txt"
#define hex014x006 "./Data/hex014x006.txt"
#define hex035x035 "./Data/hex035x035.txt"
#define hex054x045 "./Data/hex054x045.txt"
#define hex098x098 "./Data/hex098x098.txt"
#define hex113x083 "./Data/hex113x083.txt"

// change this to 1(true), and change the data below when you want to test specific starting and goal locations on startup
#define OVERRIDE_DEFAULT_STARTING_DATA 0

// Make sure your start and goal are valid locations!
#define DEFAULT_START_ROW 0
#define DEFAULT_START_COL 0
#define DEFAULT_GOAL_ROW ?
#define DEFAULT_GOAL_COL ?

//! \brief Custodian of global variables needed by all classes in this file.
struct PathSearchGlobals
{
	static const UINT SHOW_SEARCH_RUNNING = 1;

private:
	// singleton instance
	static PathSearchGlobals* instance_;

	// variables
	double min_radius_;
	UINT   flags_;

	// singleton members
	PathSearchGlobals();
	PathSearchGlobals(PathSearchGlobals const&);
	PathSearchGlobals& operator=(PathSearchGlobals const&);
	~PathSearchGlobals();

public:
	static PathSearchGlobals* getInstance();

	static void deleteInstance();

	inline double getMinTileRadius() const
	{
		return min_radius_;
	}

	inline void setMinTileRadius(double radius)
	{
		min_radius_ = radius;
	}

	inline UINT isFlagOn(UINT flag) const
	{
		return flags_ & flag;
	}

	inline void turnOn(UINT flag)
	{
		flags_ |= flag;
	}

	inline void turnOff(UINT flag)
	{
		flags_ &= ~flag;
	}
};

//! \brief Base interface between the application and the search algorithm.
struct PathSearchInterface
{
	/*This is the easiest way to get the timed search using the enter and exit code properly... 
	this is because initialize search needs to call the enter for the step and normal run searches*/
	bool								mybTimeSearch;
	bool								mybHasEntered;
	void setTimedSearch(bool isTimed) {mybTimeSearch = isTimed;}
	PathSearchInterface() { mybHasEntered  = false; }
	//! \brief This destructor is declared <code>virtual</code> to ensure that the destructors of
	//! derived classes are also invoked.
	virtual ~PathSearchInterface();

	//! \brief Returns <code>true</code> if it is okay to initialize and run the underlying search
	//! algorithm, <code>false</code> otherwise.
	virtual bool isReady() const = 0;

	//! \brief Resets any parameters that the search algorithm uses.
	virtual void resetParameters() = 0;

	//! \brief Shuts down the search algorithm.
	virtual void shutdownSearch() = 0;

	//! \brief Builds the underlying search algorithm using the contents of the specified input
	//! stream.
	virtual bool read(std::basic_ifstream<TCHAR>& input_stream) = 0;

	virtual void initialize() = 0;

	//! \brief Returns the number of parameters to be displayed by the parameter list view.
	virtual int getInputCount() const = 0;

	//! \brief Displays the relevant parameters and their current values.
	virtual void displayInput(NMLVDISPINFO* list_view_display_info) const = 0;

	//! \brief Updates any parameter values that the user has changed.
	virtual bool updateInput(NMLVDISPINFO const* list_view_display_info) = 0;

	//! \brief Resets the search algorithm.
	virtual void resetSearch() = 0;

	//! \brief Returns <code>TRUE</code> if the "Set Start" button should be enabled,
	//! <code>FALSE</code> otherwise.
	//!
	//! Override only when the search algorithm requires explicit locations as selectable items.
	virtual BOOL shouldEnableSetStart() const;

	//! \brief Updates the index of the start location.
	//!
	//! Override only when the search algorithm requires explicit locations as selectable items.
	virtual bool updateStart();

	//! \brief Returns <code>TRUE</code> if the "Set Goal" button should be enabled,
	//! <code>FALSE</code> otherwise.
	//!
	//! Override only when the search algorithm requires explicit locations.
	virtual BOOL shouldEnableSetGoal() const;

	//! \brief Updates the index of the goal location.
	//!
	//! Override only when the search algorithm requires explicit locations.
	virtual bool updateGoal();

	//! \brief Returns <code>true</code> if the user can press a key to initialize the search
	//! algorithm, <code>false</code> otherwise.
	virtual bool isInitializableByKey() const = 0;

	//! \brief Initializes the underlying search algorithm.
	virtual void initializeSearch() = 0;

	//! \brief Returns <code>true</code> if the user can press a key to run the search algorithm,
	//! <code>false</code> otherwise.
	virtual bool isRunnableByKey() const = 0;

	//! \brief Returns <code>TRUE</code> if the "Run" and "Step" buttons should be enabled,
	//! <code>FALSE</code> otherwise.
	BOOL shouldEnableRun() const;

	//! \brief Runs the search algorithm.
	virtual void runSearch(void/*long long timeslice*/) = 0;

	//! \brief step the search algorithm.
	virtual void stepSearch(void) = 0;

	//! \brief Returns <code>TRUE</code> if the "Time Run" button should be enabled,
	//! <code>FALSE</code> otherwise.
	//!
	//! Override only when the search algorithm has a clear terminating condition that can be
	//! reached during a timed run.
	virtual BOOL shouldEnableTimeRun() const;

	//! \brief Runs the search algorithm to completion and calculates the elapsed time.
	//!
	//! Override only when the search algorithm has a clear terminating condition that can be
	//! reached during a timed run.
	virtual void timeSearch();

	//! \brief Inspects the search algorithm to ensure that it has built the solution properly.
	virtual void checkSolution(HWND window_handle) const;

	//! \brief Shows the current search algortihm at work.
	//!
	//! For a regular search, this function displays the current path, all locations explored, and
	//! any locations that the search algorithm still needs to evaluate.  More exotic algorithms
	//! will exhibit different behavior.
	virtual void displaySearchProgress(POINT const& offset, int width, int height,
	                                   HDC device_context_handle) const = 0;

	//! \brief Returns <code>true</code> by default.
	//!
	//! Derived classes should return <code>false</code> if they override the
	//! <code>beginRedrawSearchProgress()</code> and <code>endRedrawSearchProgress()</code>
	//! methods.
	virtual bool needsFullRedraw() const;

	//! \brief Cleans up potential artifacts from a previous draw.
	//!
	//! Invoked before the search algorithm is executed while the
	//! <code>PathSearchGlobals::SHOW_SEARCH_RUNNING</code> flag is on.
	virtual void beginRedrawSearchProgress(POINT const& offset, int width, int height,
	                                       HDC device_context_handle) const = 0;

	//! \brief Redraws the progress of the search algorithm.
	//!
	//! Invoked after the search algorithm is executed while the
	//! <code>PathSearchGlobals::SHOW_SEARCH_RUNNING</code> flag is on.
	//!
	//! Calls <code>displaySearchProgress()</code> by default.  Override if this is inefficient.
	virtual void endRedrawSearchProgress(POINT const& offset, int width, int height,
	                                     HDC device_context_handle) const;
};

//! \brief Tile-based path planner that uses the <code>SearchLibrary</code> algorithm.
class GroundUpPathSearch : public PathSearchInterface
{
	fullsail_ai::algorithms::PathSearch search_;
	fullsail_ai::TileMap&               tile_map_;
	fullsail_ai::Tile const*            start_tile_;
	fullsail_ai::Tile const*            goal_tile_;
	LARGE_INTEGER                       frequency_;
	double                              elapsed_time_;
	unsigned int                        iteration_count_;
	int                                 start_row_;
	int                                 start_column_;
	int                                 goal_row_;
	int                                 goal_column_;
	unsigned int						myTimeStep;
	unsigned int						myFastTimeStep;
	unsigned int						myNumberofRounds;
	bool                                is_initializable_;	
	std::vector<fullsail_ai::Tile const*> path2; 

public:
	GroundUpPathSearch(fullsail_ai::TileMap& tiles);
	bool isReady() const;
	void resetParameters();
	void shutdownSearch();
	bool read(std::basic_ifstream<TCHAR>& input_stream);
	void initialize();
	int getInputCount() const;
	void displayInput(NMLVDISPINFO* list_view_display_info) const;
	bool updateInput(NMLVDISPINFO const* list_view_display_info);
	void resetSearch();
	bool isInitializableByKey() const;
	void initializeSearch();
	bool isRunnableByKey() const;
	void runSearch(void/*long long timeslice*/);
	void stepSearch(void);
	BOOL shouldEnableTimeRun() const;
	void timeSearch();
	void checkSolution(HWND window_handle) const;
	void displaySearchProgress(POINT const& offset, int width, int height,
	                           HDC device_context_handle) const;
	void beginRedrawSearchProgress(POINT const& offset, int width, int height,
	                               HDC device_context_handle) const;
};

//! \brief Police concurrent accesses via critical sections.
class CriticalSectionSynchronizer
{
	CRITICAL_SECTION cs_;

public:
	CriticalSectionSynchronizer();
	~CriticalSectionSynchronizer();
	void acquire();
	void release();
};

//! \brief Police concurrent accesses via mutices.
class MutexSynchronizer
{
	HANDLE mutex_;

public:
	MutexSynchronizer();
	~MutexSynchronizer();
	void acquire();
	void release();
};

class PathSearchApp
{
	// Pick and choose your concurrent-access police force here.
	typedef MutexSynchronizer Synchronizer;

	static const LONG      MAX_LOADSTRING = 32;
	static const LONG      MAX_BUFFER_SIZE = 256;

	// singleton instance
	static PathSearchApp * instance_;

	// application handle
	HINSTANCE              application_handle_;

	// list-view column and item counts
	int                    parameter_list_view_item_count_;

	// window, tab, list-view, and button controls
	HWND                   window_handle_;
	HWND                   globals_list_view_handle_;
	HWND                   parameter_list_view_handle_;
	HWND                   open_button_handle_;
	HWND                   reset_button_handle_;
	HWND                   start_waypoint_button_handle_;
	HWND                   goal_waypoint_button_handle_;
	HWND                   run_button_handle_;
	HWND                   step_button_handle_;
	HWND                   time_run_button_handle_;

	// custom windows
	HWND                   tile_grid_handle_;

	// double-buffering primitives
	HDC                    tile_grid_device_context_handle_;
	HDC                    tile_grid_buffer_context_handle_;
	HBITMAP                tile_grid_buffer_bitmap_handle_;

	// run-button icons
	HICON                  pause_icon_handle_;
	HICON                  play_icon_handle_;

	// coordinates, offsets, and dimensions
	int                    tile_grid_x_;
	int                    tile_grid_y_;
	int                    tile_grid_width_;
	int                    tile_grid_height_;
	int                    tile_map_width_;
	int                    tile_map_height_;
	POINT                  tile_grid_offset_;

	// synchronization primitives
	volatile bool          cannot_close_thread_handle_;
	volatile bool          should_stop_thread_handle_;
	volatile bool          needs_full_render_;
	mutable Synchronizer   planner_sync_;
	Synchronizer           thread_sync_;
	DWORD                  thread_id_;
	HANDLE                 thread_handle_;

	// path-planning components
	fullsail_ai::TileMap   ground_up_tile_map_;
	PathSearchInterface*  current_planner_;

	// the file filter string
	TCHAR                  text_filter_[12];

	// tooltip strings
	TCHAR                  open_button_text_[14];
	TCHAR                  reset_button_text_[13];
	TCHAR                  start_waypoint_button_text_[10];
	TCHAR                  goal_waypoint_button_text_[9];
	TCHAR                  run_button_text_[4];
	TCHAR                  step_button_text_[5];
	TCHAR                  time_run_button_text_[9];

	// singleton members
	PathSearchApp();
	PathSearchApp(PathSearchApp const&);
	PathSearchApp & operator=(PathSearchApp const&);
	~PathSearchApp();

public:
	static PathSearchApp * getInstance();
	static void deleteInstance();

	// Needed by the window procedure.
	inline HINSTANCE getApplicationHandle() const
	{
		return application_handle_;
	}

	// Big ol' definition.  Proceed with caution.
	BOOL initializeApplication(HINSTANCE application_handle, int n_cmd_show);

	// Thread callback helper method.
	void threadHandler();

	// Event handlers for push-button presses.
	void onReset();
	void onSetStart();
	void onSetGoal();
	void onRun();
	void onStep();
	void onTimeRun();

	// Event handler for key presses.
	bool onKeyPress(WPARAM w_param);

	// Event handler for list-views.
	LRESULT onListView(LPARAM l_param);

	// Event handler for resizing the tile grid window.
	void onSize(HWND window_handle, WPARAM w_param, LPARAM l_param);

	// Event handler for horizontal scrolling of the tile grid window.
	void onHScroll(HWND window_handle, WPARAM w_param, LPARAM l_param);

	// Event handler for vertical scrolling of the tile grid window.
	void onVScroll(HWND window_handle, WPARAM w_param, LPARAM l_param);

	// Event handler for file opening.
	void onFileOpen(HWND window_handle);

	// Event handler for rendering the path planner.
	void paintTileGrid(HDC device_context_handle) const;

private:
	// Helper methods for updating the UI.
	void resetOffsets_();
	void showRunStopped_();
	void updateRunButtons_();
	void updateButtons_();
	void updateParameterListView_();
	void updateTileGrid_();

	// Helper method for file opening.
	void openFile_(HWND window_handle, TCHAR* file_name);

	// Unsynchronized helper methods for event handlers.
	void reset_();
	bool run_();
	void step_();
	void timeRun_();

	// Helper methods for key presses.
	void resetByKey_();
	void runByKey_();
	void stepByKey_();
	void timeRunByKey_();

	// Helper methods for rendering the path planner.
	void renderFull_();
	void doubleBufferTileGrid_(HDC device_context_handle, HDC buffer_context_handle) const;
	void paintAllTileGrid_(HDC device_context_handle, HDC buffer_context_handle) const;
};

