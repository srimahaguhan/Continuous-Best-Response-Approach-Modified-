#include "single_agent_ecbs.h"
#include <cstring>
#include <climits>
#include <vector>
#include <list>
#include <utility>

#include <boost/heap/fibonacci_heap.hpp>
//#include <google/dense_hash_map>

#include "Node.h"

//using google::dense_hash_map;      // namespace where class lives by default
using std::cout;
using std::endl;
using boost::heap::fibonacci_heap;


SingleAgentECBS::SingleAgentECBS(const vector<vector<int> > &cons_paths, const vector<int> &my_heuristic, const vector<bool> &my_map,
	int ag_id, int start_location, int goal_location, int col, int curr_time, int max_time) :
		cons_paths(cons_paths), my_heuristic(my_heuristic), my_map(my_map), ag_id(ag_id), start_location(start_location), goal_location(goal_location), curr_time(curr_time), 
		num_expanded(0), num_generated(0), path_cost(0), lower_bound(0), min_f_val(0), num_non_hwy_edges(0), max_time(max_time)
	{
 
  this->map_size = my_map.size();
  // initialize allNodes_table (hash table)
  //empty_node = new Node();
  //empty_node->loc = -1;
  //deleted_node = new Node();
  //deleted_node->loc = -2;
  //allNodes_table.set_empty_key(empty_node);
  //allNodes_table.set_deleted_key(deleted_node);
  actions_offset[0] = 0; actions_offset[1] = -col; actions_offset[2] = 1; actions_offset[3] = col; actions_offset[4] = -1; // [WAIT, NORTH, EAST, SOUTH, WEST]

}


void SingleAgentECBS::updatePath(Node* goal) {
  path.clear();
  Node* curr = goal;
  // cout << "   UPDATING Path for one agent to: ";
  while (curr->timestep != 0) {
	  path.push_back(curr->loc);
        //cout << curr->loc << endl;
    curr = curr->parent;
  }
  path.push_back(start_location);
  reverse(path.begin(), path.end());
  path_cost = goal->g_val;
}

inline void SingleAgentECBS::releaseClosedListNodes(map<unsigned int, Node*> &allNodes_table) {
   map<unsigned int, Node*>::iterator it;
  for (it=allNodes_table.begin(); it != allNodes_table.end(); it++) {
    delete ( (*it).second );  // Node* s = (*it).first; delete (s);
  }
  allNodes_table.clear();
}


// iterate over the constraints ( cons[t] is a list of all constraints for timestep t) and return the latest
// timestep which has a constraint involving the goal location
int SingleAgentECBS::extractLastGoalTimestep(int goal_location, const vector< list< pair<int, int> > >* cons) {
  if (cons != NULL) {
    for ( int t = static_cast<int>(cons->size())-1; t > 0; t-- ) {
      for (list< pair<int, int> >::const_iterator it = cons->at(t).begin(); it != cons->at(t).end(); ++it) {
        // $$$: in the following if, do we need to check second (maybe cannot happen in edge constraints?)
        if ((*it).first == goal_location || (*it).second == goal_location) {
          return (t);
        }
      }
    }
  }
  return -1;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// input: curr_id (location at time next_timestep-1) ; next_id (location at time next_timestep); next_timestep
//        cons[timestep] is a list of <loc1,loc2> of (vertex/edge) constraints for that timestep.
inline bool SingleAgentECBS::isConstrained(int curr_loc, int next_loc, int next_timestep, const vector< list< pair<int, int> > >* cons) 
{
	//check whether it is a block
	if (!my_map[next_loc]) return true;

	//cheack constraints with DELIVER agents
	for (unsigned int i = 0; i < cons_paths.size(); i++)
	{
		if (cons_paths[i][curr_time + next_timestep] == next_loc)
			return true; //vertext collision
		else if (cons_paths[i][curr_time + next_timestep] == curr_loc &&cons_paths[i][curr_time + next_timestep - 1] == next_loc)
			return true; //edge collision
	}

  //  cout << "check if ID="<<id<<" is occupied at TIMESTEP="<<timestep<<endl;
  if (cons == NULL)
    return false;

  // check vertex constraints (being in next_loc at next_timestep is disallowed)
  if ( next_timestep < static_cast<int>(cons->size()) ) {
    for ( list< pair<int, int> >::const_iterator it = cons->at(next_timestep).begin(); it != cons->at(next_timestep).end(); ++it ) {
      if ( (*it).second == -1 ) {
        if ( (*it).first == next_loc ) {
          return true;
        }
      }
    }
  }

  // check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
  if ( next_timestep > 0 && next_timestep - 1 < static_cast<int>(cons->size()) ) {
    for ( list< pair<int, int> >::const_iterator it = cons->at(next_timestep-1).begin(); it != cons->at(next_timestep-1).end(); ++it ) {
      if ( (*it).first == curr_loc && (*it).second == next_loc ) {
        return true;
      }
    }
  }

  return false;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int SingleAgentECBS::numOfConflictsForStep(int curr_id, int next_id, int next_timestep, bool* res_table, int max_plan_len) {
  int retVal = 0;
  if (next_timestep >= max_plan_len) {
    // check vertex constraints (being at an agent's goal when he stays there because he is done planning)
    if ( res_table[next_id + (max_plan_len-1)*map_size] == true )
      retVal++;
    // Note -- there cannot be edge conflicts when other agents are done moving
  } else {
    // check vertex constraints (being in next_id at next_timestep is disallowed)
    if ( res_table[next_id + next_timestep*map_size] == true )
      retVal++;
    // check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
    // which means that res_table is occupied with another agent for [curr_id,next_timestep] and [next_id,next_timestep-1]
    if ( res_table[curr_id + next_timestep*map_size] && res_table[next_id + (next_timestep-1)*map_size] )
      retVal++;
  }
  //  cout << "#CONF=" << retVal << " ; For: curr_id=" << curr_id << " , next_id=" << next_id << " , next_timestep=" << next_timestep
  //       << " , max_plan_len=" << max_plan_len << endl;
  return retVal;
}

// $$$ -- is there a more efficient way to do that?
void SingleAgentECBS::updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight) {
  //  cout << "Update Focal: (old_LB=" << old_lower_bound << " ; new_LB=" << new_lower_bound << endl;;
  for (Node* n : open_list) {
    //    cout << "   Considering " << n << " , " << *n << endl;
    if ( n->getFVal() > old_lower_bound &&
         n->getFVal() <= new_lower_bound ) {
      //      cout << "      Added (n->f-val=" << n->getFVal() << ")" << endl;
      n->focal_handle = focal_list.push(n);
    }
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists
bool SingleAgentECBS::findPath(double f_weight, const vector < list< pair<int, int> > >* constraints, bool* res_table, size_t max_plan_len) {
  // clear data structures if they had been used before
  // (note -- nodes are deleted before findPath returns)
  open_list.clear();
  focal_list.clear();
  //allNodes_table.clear();
  map<unsigned int, Node*> allNodes_table; //int=g_val*map_size+loc
  num_expanded = 0;
  num_generated = 0;

  //hashtable_t::iterator it;  // will be used for find()

  // generate start and add it to the OPEN list
  Node* start = new Node(start_location, 0, my_heuristic[start_location], NULL, 0, 0, false);
  num_generated++;
  start->open_handle = open_list.push(start);
  start->focal_handle = focal_list.push(start);
  start->in_openlist = true;
  //allNodes_table[start] = start;
  allNodes_table.insert(make_pair(start_location, start)); //g_val=0
  min_f_val = start->getFVal();
  lower_bound = f_weight * min_f_val;

  int lastGoalConsTime = extractLastGoalTimestep(goal_location, constraints);

  while ( !focal_list.empty() ) {
    //    cout << "|F|=" << focal_list.size() << " ; |O|=" << open_list.size() << endl;
    Node* curr = focal_list.top(); focal_list.pop();
    //    cout << "Current FOCAL bound is " << lower_bound << endl;
    //    cout << "POPPED FOCAL's HEAD: (" << curr << ") " << (*curr) << endl;
    open_list.erase(curr->open_handle);
    //    cout << "DELETED" << endl; fflush(stdout);
    curr->in_openlist = false;
     num_expanded++;

    // check if the popped node is a goal
    if (curr->loc == goal_location && curr->timestep > lastGoalConsTime) 
	{
		//chack whether it can be held
		bool hold = true;
		for (unsigned int ag = 0; ag < cons_paths.size() && hold == true; ag++)
		{
			for (unsigned int t = curr->timestep + curr_time + 1; t < cons_paths[ag].size() && hold == true; t++)
			{
				if (cons_paths[ag][t] == curr->loc)
				{
					hold = false;
				}
			}
		}
		if (hold)
		{
			updatePath(curr);
			releaseClosedListNodes(allNodes_table);
			return true;
		}
    }
	// If current node is not goal, generate successors
	for (int direction = 0; direction < 5; direction++)
	{
		int next_id = curr->loc + actions_offset[direction];
		int next_timestep = curr->timestep + 1;
		
		if (!isConstrained(curr->loc, next_id, next_timestep, constraints)) {
			// compute cost to next_id via curr node
			double cost = 1;
			// if (action == MapLoader::WAIT_ACTION) {
			//   cost = 0.5;
			// }
			double next_g_val = curr->g_val + cost;
			double next_h_val = my_heuristic[next_id];
			int next_internal_conflicts = 0;
			if (max_plan_len > 0)  // check if the reservation table is not empty (that is tha max_length of any other agent's plan is > 0)
				next_internal_conflicts = curr->num_internal_conf + numOfConflictsForStep(curr->loc, next_id, next_timestep, res_table, max_plan_len);
			// generate (maybe temporary) node
			Node* next = new Node(next_id, next_g_val, next_h_val, curr, next_timestep, next_internal_conflicts, false);
			       // cout << "   NEXT(" << next << ")=" << *next << endl;
			// try to retrieve it from the hash table
			//it = allNodes_table.find(next);
			map<unsigned int, Node* >::iterator it = allNodes_table.find(next->loc + next->g_val*my_map.size());

			if (it == allNodes_table.end() && next->g_val<max_time - curr_time) {  // add the newly generated node to open_list and hash table
				//          cout << "   ADDING it as new." << endl;
				next->open_handle = open_list.push(next);
				next->in_openlist = true;
				num_generated++;
				if (next->getFVal() <= lower_bound)
					next->focal_handle = focal_list.push(next);
				//allNodes_table[next] = next;
				allNodes_table.insert(pair<unsigned int, Node*>(next->loc + next->g_val*my_map.size(), next));

			}
			else {  // update existing node's if needed (only in the open_list)
				delete(next);  // not needed anymore -- we already generated it before
				Node* existing_next = (*it).second;
				//          cout << "Actually next exists. It's address is " << existing_next << endl;
				if (existing_next->in_openlist == true) {  // if its in the open list
					if (existing_next->getFVal() > next_g_val + next_h_val ||
						(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf > next_internal_conflicts)) {
						// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
						//              cout << "   UPDATE its f-val in OPEN (decreased or less #conflicts)" << endl;
						//              cout << "   Node state before update: " << *existing_next;
						bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
						bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
						bool update_open = false;
						if ((next_g_val + next_h_val) <= lower_bound) {  // if the new f-val qualify to be in FOCAL
							if (existing_next->getFVal() > lower_bound)
								add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
							else
								update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
						}
						if (existing_next->getFVal() > next_g_val + next_h_val)
							update_open = true;
						// update existing node
						existing_next->g_val = next_g_val;
						existing_next->h_val = next_h_val;
						existing_next->parent = curr;
						existing_next->num_internal_conf = next_internal_conflicts;
						//              cout << "   Node state after update: " << *existing_next;
						if (update_open) {
							open_list.increase(existing_next->open_handle);  // increase because f-val improved
							//                cout << "     Increased in OPEN" << endl;
						}
						if (add_to_focal) {
							existing_next->focal_handle = focal_list.push(existing_next);
							//                cout << "     Inserted to FOCAL" << endl;
						}
						if (update_in_focal) {
							focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
							//                cout << "     Updated in FOCAL" << endl;
						}
					}
					//            cout << "   Do NOT update in OPEN (f-val for this node increased or stayed the same and has more conflicts)" << endl;
				}
				else {  // if its in the closed list (reopen)
					if (existing_next->getFVal() > next_g_val + next_h_val ||
						(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf > next_internal_conflicts)) {
						// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
						//              cout << "   Reinsert it to OPEN" << endl;
						//              cout << "   Node state before update: " << *existing_next;
						existing_next->g_val = next_g_val;
						existing_next->h_val = next_h_val;
						existing_next->parent = curr;
						existing_next->num_internal_conf = next_internal_conflicts;
						existing_next->open_handle = open_list.push(existing_next);
						existing_next->in_openlist = true;
						//              cout << "   Node state after update: " << *existing_next;
						if (existing_next->getFVal() <= lower_bound) {
							existing_next->focal_handle = focal_list.push(existing_next);
							//                cout << "     Inserted to FOCAL" << endl;
						}
					}
					//            cout << "   Do NOT reopen" << endl;
				}  // end update a node in closed list
			}  // end update an existing node
		}  // end if case for grid not blocked

	}  // end for loop that generates successors
    // update FOCAL if min f-val increased
    if (open_list.size() == 0)  // in case OPEN is empty, no path found...
      return false;
    Node* open_head = open_list.top();
    if ( open_head->getFVal() > min_f_val ) {
      double new_min_f_val = open_head->getFVal();
      double new_lower_bound = f_weight * new_min_f_val;
      /*
        cout << "LL FOCAL UPDATE! Old-f-min=" << min_f_val << " ; Old-LB=" << lower_bound << endl;
        cout << "OPEN: ";
        for (Node* n : open_list)
        cout << n << " , ";
        cout << endl;
        cout << "FOCAL: ";
        for (Node* n : focal_list)
        cout << n << " , ";
        cout << endl;
      */
      updateFocalList(lower_bound, new_lower_bound, f_weight);
      min_f_val = new_min_f_val;
      lower_bound = new_lower_bound;
      /*
        cout << "   New-f-min=" << min_f_val << " ; New-LB=" << lower_bound << endl;
        cout << "FOCAL: ";
        for (Node* n : focal_list)
        cout << n << " , ";
        cout << endl;
      */
    }
  }  // end while loop
  // no path found
  path.clear();
  releaseClosedListNodes(allNodes_table);
  return false;
}

SingleAgentECBS::~SingleAgentECBS() {
  //delete (empty_node);
  //delete (deleted_node);
}
