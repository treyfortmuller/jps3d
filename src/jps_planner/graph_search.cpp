#include <jps_planner/jps_planner/graph_search.h>
#include <cmath>
#include <chrono>

using namespace JPS;

// deleted 2D graph search constructor

// TODO: check this constructor, what is this about neighbors???
GraphSearch::GraphSearch(const char *cMap, int xDim, int yDim, int zDim, double eps, bool verbose) : cMap_(cMap), xDim_(xDim), yDim_(yDim), zDim_(zDim), eps_(eps), verbose_(verbose)
{
  hm_.resize(xDim_ * yDim_ * zDim_);
  seen_.resize(xDim_ * yDim_ * zDim_, false);

  // ns_ is not used in JPS its only for A*

  // Set 3D neighbors
  for (int x = -1; x <= 1; x++)
  {
    for (int y = -1; y <= 1; y++)
    {
      for (int z = -1; z <= 1; z++)
      {
        if (x == 0 && y == 0 && z == 0)
          continue;
        ns_.push_back(std::vector<int>{x, y, z});
      }
    }
  }
  jn3d_ = std::make_shared<JPS3DNeib>();
}

// deleted 2D coordToId

inline int GraphSearch::coordToId(int x, int y, int z) const
{
  return x + y * xDim_ + z * xDim_ * yDim_;
}

// deleted 2D isFree

inline bool GraphSearch::isFree(int x, int y, int z) const
{
  return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && z >= 0 && z < zDim_ &&
         cMap_[coordToId(x, y, z)] == val_free_;
}

// deleted 2D isOccupied

inline bool GraphSearch::isOccupied(int x, int y, int z) const
{
  return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && z >= 0 && z < zDim_ &&
         cMap_[coordToId(x, y, z)] > val_free_;
}

// deleted 2D getHeur

inline double GraphSearch::getHeur(int x, int y, int z) const
{
  // eps_ is the "epsilon" is the heuristic weight, which defaults to 1
  return eps_ * std::sqrt((x - xGoal_) * (x - xGoal_) + (y - yGoal_) * (y - yGoal_) + (z - zGoal_) * (z - zGoal_));
}

// deleted 2D plan prep function here

// This is the 3D JPS plan function that we care about
bool GraphSearch::plan(int xStart, int yStart, int zStart, int xGoal, int yGoal, int zGoal, bool useJps, int maxExpand)
{
  use_2d_ = false;
  pq_.clear();
  path_.clear();
  hm_.resize(xDim_ * yDim_ * zDim_);
  seen_.resize(xDim_ * yDim_ * zDim_, false);

  // Set jps
  use_jps_ = useJps;

  // Set goal
  int goal_id = coordToId(xGoal, yGoal, zGoal);
  xGoal_ = xGoal;
  yGoal_ = yGoal;
  zGoal_ = zGoal;
  // Set start node
  int start_id = coordToId(xStart, yStart, zStart);
  StatePtr currNode_ptr = std::make_shared<State>(State(start_id, xStart, yStart, zStart, 0, 0, 0));
  currNode_ptr->g = 0;
  currNode_ptr->h = getHeur(xStart, yStart, zStart);

  // This is the 3rd nested plan function that gets called... down we go
  return plan(currNode_ptr, maxExpand, start_id, goal_id);
}

// ***************** THE 3D JPS PLAN FUNCTION *****************

// THIS is finally the actual business logic of 3D JPS, all the other higher level plan functions were just
// wrappers doing sanity checks and setting up the state for this, its 100 lines long
bool GraphSearch::plan(StatePtr &currNode_ptr, int maxExpand, int start_id, int goal_id)
{

  printf("Entered the 3D JPS Plan Business logic.\n---\n\n");

  // Insert start node - note he uses a BOOST priority queue whereas I use a std::priority_queue
  // ALSO: he maintains a priority queue of pointers to nodes rather than a priority queue of node IDs like me
  currNode_ptr->heapkey = pq_.push(currNode_ptr);
  currNode_ptr->opened = true;
  hm_[currNode_ptr->id] = currNode_ptr;
  seen_[currNode_ptr->id] = true;

  int expand_iteration = 0;
  while (true)
  {
    expand_iteration++;
    // get element with smallest cost
    currNode_ptr = pq_.top();
    pq_.pop();
    currNode_ptr->closed = true; // Add to closed list

    if (currNode_ptr->id == goal_id)
    {
      if (verbose_)
        printf("Goal Reached!!!!!!\n\n");
      break;
    }

    // Here's the current node we're expanding!
    printf("A* expansion on: %d, %d\n", currNode_ptr->x, currNode_ptr->y);

    std::vector<int> succ_ids;
    std::vector<double> succ_costs;

    // Get successors
    // deleted check against whether we need A* "successors" or JPS succesors

    // We'll always be getting the JPS successors, time this function
    auto getSuccStart = std::chrono::steady_clock::now();

    getJpsSucc(currNode_ptr, succ_ids, succ_costs);

    auto getSuccEnd = std::chrono::steady_clock::now();
    auto succTime = std::chrono::duration_cast<std::chrono::milliseconds>(getSuccEnd - getSuccStart).count();
    printf("Get Successors took: %ld ms\n", succTime);

    // if(verbose_)
    // printf("size of succs: %zu\n", succ_ids.size());

    // Process successors
    for (int s = 0; s < (int)succ_ids.size(); s++)
    {
      //see if we can improve the value of succstate
      StatePtr &child_ptr = hm_[succ_ids[s]];
      double tentative_gval = currNode_ptr->g + succ_costs[s];

      // This guy does the check to see if he can lower the g score when he re-expands a node which
      // I don't think is necessary when using a consistent heuristic.
      if (tentative_gval < child_ptr->g) // We must always drop in here if there's no g-value set
      {
        child_ptr->parentId = currNode_ptr->id; // Assign new parent
        child_ptr->g = tentative_gval;          // Update gval

        //double fval = child_ptr->g + child_ptr->h;

        // if currently in OPEN, update - I don't think this is necessary
        if (child_ptr->opened && !child_ptr->closed)
        {
          pq_.increase(child_ptr->heapkey); // update heap
          child_ptr->dx = (child_ptr->x - currNode_ptr->x);
          child_ptr->dy = (child_ptr->y - currNode_ptr->y);
          child_ptr->dz = (child_ptr->z - currNode_ptr->z);
          if (child_ptr->dx != 0)
            child_ptr->dx /= std::abs(child_ptr->dx);
          if (child_ptr->dy != 0)
            child_ptr->dy /= std::abs(child_ptr->dy);
          if (child_ptr->dz != 0)
            child_ptr->dz /= std::abs(child_ptr->dz);
        }
        // if currently in CLOSED
        else if (child_ptr->opened && child_ptr->closed)
        {
          printf("ASTAR ERROR!\n");
        }
        else // new node, add to heap
        {
          //printf("add to open set: %d, %d\n", child_ptr->x, child_ptr->y);
          child_ptr->heapkey = pq_.push(child_ptr);
          child_ptr->opened = true;
        }
      } // This if block is still necessary since he does g value assignment here
    }   // Process successors

    if (maxExpand > 0 && expand_iteration >= maxExpand)
    {
      if (verbose_)
        printf("MaxExpandStep [%d] Reached!!!!!!\n\n", maxExpand);
      return false;
    }

    if (pq_.empty())
    {
      if (verbose_)
        printf("Priority queue is empty!!!!!!\n\n");
      return false;
    }
  }

  if (verbose_)
  {
    // Kinda dumb print statement because we already print the total path cost
    // printf("goal g: %f, h: %f!\n", currNode_ptr->g, currNode_ptr->h);

    // A* expanded nodes
    printf("A* expanded %d nodes!\n", expand_iteration);
  }

  // Now recover the path given the current node's pointer and the ID of the start node
  // time this.
  auto recoverTimeStart = std::chrono::steady_clock::now();
  path_ = recoverPath(currNode_ptr, start_id);
  auto recoverTimeEnd = std::chrono::steady_clock::now();
  auto recoverTime = std::chrono::duration_cast<std::chrono::nanoseconds>(recoverTimeEnd - recoverTimeStart).count();
  printf("Path recovery took: %ld ns\n", recoverTime);
  // std::cout << "Path smoothing took: " << recoverTime << " ns" << std::endl;

  // We've successfully found a path so return true
  return true;
}

// ***************** THE 3D JPS PLAN FUNCTION ENDS *****************

std::vector<StatePtr> GraphSearch::recoverPath(StatePtr node, int start_id)
{
  std::vector<StatePtr> path;
  path.push_back(node);
  while (node && node->id != start_id)
  {
    node = hm_[node->parentId];
    path.push_back(node);
  }

  return path;
}

// *********** JPS 3D GET SUCCESSORS BUSINESS LOGIC *************

// 3D JUMP POINT SEARCH GET SUCCESSORS BUSINESS LOGIC
void GraphSearch::getJpsSucc(const StatePtr &curr, std::vector<int> &succ_ids, std::vector<double> &succ_costs)
{

  // 3D JPS get successors
  // essentially computing the motion diagonal order - he uses 1 for a straight move, this is a way faster computation
  const int norm1 = std::abs(curr->dx) + std::abs(curr->dy) + std::abs(curr->dz);

  // TODO: I don't understand what these are, this is critical
  // static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};

  int num_neib = jn3d_->nsz[norm1][0];
  int num_fneib = jn3d_->nsz[norm1][1];

  printf("     norm: %d\n", norm1);
  printf(" num_neib: %d\n", num_neib);
  printf("num_fneib: %d\n", num_fneib);

  // When moving in a cardinal direction there is 1 natural neighbor and 8 forced neighbors
  // num_neib + num_fneib is 9

  // get the unique ID associated with the motion direction
  int id = (curr->dx + 1) + 3 * (curr->dy + 1) + 9 * (curr->dz + 1);

  // A for loop from 0 to the sum of the neighbors and the forced neighbors
  for (int dev = 0; dev < num_neib + num_fneib; ++dev)
  {
    int new_x, new_y, new_z;
    int dx, dy, dz;

    // Are we checking the natural neighbors or the forced neighbors
    if (dev < num_neib)
    {
      // We're checking the natural neighbors

      // Get the elements of the direction vector to this neighbor
      // this must be looking up the values of a direction vector for each neighbor
      dx = jn3d_->ns[id][0][dev]; // TODO: how does this array work?
      dy = jn3d_->ns[id][1][dev];
      dz = jn3d_->ns[id][2][dev];

      // Interesting... we only ever seem to check if we don't detect a jump point
      // timing this, this is apparently crazy stupid fast

      auto jumpTimeStart = std::chrono::steady_clock::now();

      auto jumpResult = jump(curr->x, curr->y, curr->z,
                             dx, dy, dz, new_x, new_y, new_z);

      auto jumpTimeEnd = std::chrono::steady_clock::now();
      auto jumpTime = std::chrono::duration_cast<std::chrono::nanoseconds>(jumpTimeEnd - jumpTimeStart).count();
      printf("JUMP TOOK: %ld ns\n", jumpTime);

      if (!jumpResult)
      {
        continue;
      }
    }
    else
    {
      // we are checking the forced neighbors

      // get the forced neighbor relative location out of f1
      // NOTE: Sikang calls these forced neighbors, but I think f1 is full of the blocking obstacle positions
      int nx = curr->x + jn3d_->f1[id][0][dev - num_neib];
      int ny = curr->y + jn3d_->f1[id][1][dev - num_neib];
      int nz = curr->z + jn3d_->f1[id][2][dev - num_neib];
      if (isOccupied(nx, ny, nz)) // if the blocking obstacle is occupied, go expand in the direction of the forced neighbor
      {
        // f2 actually has forced neighbor positions due to a blocking obstacle position in f1
        dx = jn3d_->f2[id][0][dev - num_neib];
        dy = jn3d_->f2[id][1][dev - num_neib];
        dz = jn3d_->f2[id][2][dev - num_neib];
        // jump in the direction of the forced neighbor
        if (!jump(curr->x, curr->y, curr->z,
                  dx, dy, dz, new_x, new_y, new_z))
          continue;
      }
      else
        continue;
    } // Now we've completed all the jumping we need to do

    // We're still inside the for loop iterating over a range from 0 to the sum
    // of the natural and forced neighbors (I need to watch out for his misusing the term "forced neighbors")

    // because we've arrived down here we know that we have a jump point successor in the direction of new_x, new_y, new_z
    // so that coordinate should be added as a jump point, do the processing below

    // get the coordinate of the successor
    int new_id = coordToId(new_x, new_y, new_z);
    if (!seen_[new_id])
    {
      seen_[new_id] = true;
      hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, new_z, dx, dy, dz);
      hm_[new_id]->h = getHeur(new_x, new_y, new_z);
    }

    // push the successor on!
    succ_ids.push_back(new_id);
    succ_costs.push_back(std::sqrt((new_x - curr->x) * (new_x - curr->x) +
                                   (new_y - curr->y) * (new_y - curr->y) +
                                   (new_z - curr->z) * (new_z - curr->z)));
  }
  // }
}

// *********** END JPS 3D GET SUCCESSORS BUSINESS LOGIC *************

// deleted the 2D version of the jump function that existed here

// This is the jump function we care about!
// Note he just returns true when a node is a jump point, I return an optional size_t indicating where the jump point is which must be
bool GraphSearch::jump(int x, int y, int z, int dx, int dy, int dz, int &new_x, int &new_y, int &new_z)
{
  // auto addStart = std::chrono::steady_clock::now();

  // I have similar code that gets the next neighbor
  new_x = x + dx;
  new_y = y + dy;
  new_z = z + dz;

  // auto addEnd = std::chrono::steady_clock::now();
  // auto addTime = std::chrono::duration_cast<std::chrono::nanoseconds>(addEnd - addStart).count();
  // printf("Add time took: %ld ns\n", addTime);

  // Output the coordinate we're jumping with
  // printf("Jumping with [%d, %d, %d] in direction [%d, %d, %d]\n", new_x, new_y, new_z, dx, dy, dz);

  // I have a similar check
  if (!isFree(new_x, new_y, new_z))
    return false;

  // I have a similar check
  if (new_x == xGoal_ && new_y == yGoal_ && new_z == zGoal_)
    return true;

  // here I would grab ALL the neighbors of the current node, then "computeForcedNeighbors" with that extraction as an argument

  // auto forcedStart = std::chrono::steady_clock::now();
  bool hasForcedResult = hasForced(new_x, new_y, new_z, dx, dy, dz);
  // auto forcedEnd = std::chrono::steady_clock::now();
  // auto forcedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(forcedEnd - forcedStart).count();
  // printf("hasForced time took: %ld ns\n", forcedTime);

  if (hasForcedResult)
    // This node has forced neighbors so it is a jump point
    return true;

  const int id = (dx + 1) + 3 * (dy + 1) + 9 * (dz + 1);
  const int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
  int num_neib = jn3d_->nsz[norm1][0];
  for (int k = 0; k < num_neib - 1; ++k)
  {
    int new_new_x, new_new_y, new_new_z;
    if (jump(new_x, new_y, new_z,
             jn3d_->ns[id][0][k], jn3d_->ns[id][1][k], jn3d_->ns[id][2][k],
             new_new_x, new_new_y, new_new_z))
      return true;
  }

  return jump(new_x, new_y, new_z, dx, dy, dz, new_x, new_y, new_z);
}

// Deleted the 2D version of hasForced here

inline bool GraphSearch::hasForced(int x, int y, int z, int dx, int dy, int dz)
{
  int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);

  // How can you get an ID just from the direction vector, doesn't seem like this should be enough information
  // Printing the direction vectors
  // printf("[dx, dy, dz]: [%d, %d, %d]\n", dx, dy, dz);
  // dx, dy, dz seem to form direction vectors in my sense of the word

  // This is a direction ID
  int id = (dx + 1) + 3 * (dy + 1) + 9 * (dz + 1);
  // This ID is the same when the direction vector is the same, maybe this is not an actual cell ID but an
  // ID associated with a direction?
  // always 14 for a positive x cardinal direction vector
  // always 16 for a positive y cardinal direction vector

  // printf("id: %d\n", id);

  // Switch on the motion diagonal order!
  switch (norm1)
  {
  case 1:
    // 1-d move, check 8 neighbors
    for (int fn = 0; fn < 8; ++fn)
    {
      // This is a 1D move, we have eight blocking neighbor positions to check
      // blocking obstacle positions?
      // for each neighbor, compute its x, y, z coordinate position from this map and check if its occupied, if any one of them is we can
      // short circuit, early return, and say yes. This node has forced neighbors (MAYBE EVEN IF THOSE FOCED NEIGHBORS ARE NOT PASSABLE)

      // He uses the ID computed above (directly associated with a directon vector) to index an array
      // This simulates the effect of a hashing function, we're just coming up with a value we an index an array with
      // (I used a map though)

      // ? What is f1?
      int nx = x + jn3d_->f1[id][0][fn]; // id indexes the direction we're moving
      int ny = y + jn3d_->f1[id][1][fn]; // 0, 1, 2 indexes the x, y, or z element of the neighbor
      int nz = z + jn3d_->f1[id][2][fn]; // fn indexes the neighbor we're looking at... seems weird

      // ^^^ I would have done f1[id][fn][0] ???

      if (isOccupied(nx, ny, nz))
        return true;
    }
    // None of the blocking obstacle positions are impassable so we can prune everything and this node has no forced neighbors
    return false;
  case 2:
    // 2-d move, check 8 neighbors
    for (int fn = 0; fn < 8; ++fn)
    {
      int nx = x + jn3d_->f1[id][0][fn];
      int ny = y + jn3d_->f1[id][1][fn];
      int nz = z + jn3d_->f1[id][2][fn];
      if (isOccupied(nx, ny, nz))
        return true;
    }
    return false;
  case 3:
    // 3-d move, check 6 neighbors
    // TODO! I count 7 obstacle positions in this case what are these 6?
    for (int fn = 0; fn < 6; ++fn)
    {
      int nx = x + jn3d_->f1[id][0][fn];
      int ny = y + jn3d_->f1[id][1][fn];
      int nz = z + jn3d_->f1[id][2][fn];
      if (isOccupied(nx, ny, nz))
        return true;
    }
    return false;
  default:
    // probably in the case of a motion diagonal order of 0, return that it does not have forced neighbors
    return false;
  }
}

std::vector<StatePtr> GraphSearch::getPath() const
{
  return path_;
}

std::vector<StatePtr> GraphSearch::getOpenSet() const
{
  std::vector<StatePtr> ss;
  for (const auto &it : hm_)
  {
    if (it && it->opened && !it->closed)
      ss.push_back(it);
  }
  return ss;
}

std::vector<StatePtr> GraphSearch::getCloseSet() const
{
  std::vector<StatePtr> ss;
  for (const auto &it : hm_)
  {
    if (it && it->closed)
      ss.push_back(it);
  }
  return ss;
}

std::vector<StatePtr> GraphSearch::getAllSet() const
{
  std::vector<StatePtr> ss;
  for (const auto &it : hm_)
  {
    if (it)
      ss.push_back(it);
  }
  return ss;
}

// deleted a bunch of JPS 2D neighbor stuff

// computed at compile time
constexpr int JPS3DNeib::nsz[4][2];

JPS3DNeib::JPS3DNeib()
{
  int id = 0;
  for (int dz = -1; dz <= 1; ++dz)
  {
    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dx = -1; dx <= 1; ++dx)
      {
        int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
        for (int dev = 0; dev < nsz[norm1][0]; ++dev)
          Neib(dx, dy, dz, norm1, dev,
               ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
        for (int dev = 0; dev < nsz[norm1][1]; ++dev)
        {
          FNeib(dx, dy, dz, norm1, dev,
                f1[id][0][dev], f1[id][1][dev], f1[id][2][dev],
                f2[id][0][dev], f2[id][1][dev], f2[id][2][dev]);
        }
        id++;
      }
    }
  }
}

void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev,
                     int &tx, int &ty, int &tz)
{
  switch (norm1)
  {
  case 0:
    switch (dev)
    {
    case 0:
      tx = 1;
      ty = 0;
      tz = 0;
      return;
    case 1:
      tx = -1;
      ty = 0;
      tz = 0;
      return;
    case 2:
      tx = 0;
      ty = 1;
      tz = 0;
      return;
    case 3:
      tx = 1;
      ty = 1;
      tz = 0;
      return;
    case 4:
      tx = -1;
      ty = 1;
      tz = 0;
      return;
    case 5:
      tx = 0;
      ty = -1;
      tz = 0;
      return;
    case 6:
      tx = 1;
      ty = -1;
      tz = 0;
      return;
    case 7:
      tx = -1;
      ty = -1;
      tz = 0;
      return;
    case 8:
      tx = 0;
      ty = 0;
      tz = 1;
      return;
    case 9:
      tx = 1;
      ty = 0;
      tz = 1;
      return;
    case 10:
      tx = -1;
      ty = 0;
      tz = 1;
      return;
    case 11:
      tx = 0;
      ty = 1;
      tz = 1;
      return;
    case 12:
      tx = 1;
      ty = 1;
      tz = 1;
      return;
    case 13:
      tx = -1;
      ty = 1;
      tz = 1;
      return;
    case 14:
      tx = 0;
      ty = -1;
      tz = 1;
      return;
    case 15:
      tx = 1;
      ty = -1;
      tz = 1;
      return;
    case 16:
      tx = -1;
      ty = -1;
      tz = 1;
      return;
    case 17:
      tx = 0;
      ty = 0;
      tz = -1;
      return;
    case 18:
      tx = 1;
      ty = 0;
      tz = -1;
      return;
    case 19:
      tx = -1;
      ty = 0;
      tz = -1;
      return;
    case 20:
      tx = 0;
      ty = 1;
      tz = -1;
      return;
    case 21:
      tx = 1;
      ty = 1;
      tz = -1;
      return;
    case 22:
      tx = -1;
      ty = 1;
      tz = -1;
      return;
    case 23:
      tx = 0;
      ty = -1;
      tz = -1;
      return;
    case 24:
      tx = 1;
      ty = -1;
      tz = -1;
      return;
    case 25:
      tx = -1;
      ty = -1;
      tz = -1;
      return;
    }
  case 1:
    tx = dx;
    ty = dy;
    tz = dz;
    return;
  case 2:
    switch (dev)
    {
    case 0:
      if (dz == 0)
      {
        tx = 0;
        ty = dy;
        tz = 0;
        return;
      }
      else
      {
        tx = 0;
        ty = 0;
        tz = dz;
        return;
      }
    case 1:
      if (dx == 0)
      {
        tx = 0;
        ty = dy;
        tz = 0;
        return;
      }
      else
      {
        tx = dx;
        ty = 0;
        tz = 0;
        return;
      }
    case 2:
      tx = dx;
      ty = dy;
      tz = dz;
      return;
    }
  case 3:
    switch (dev)
    {
    case 0:
      tx = dx;
      ty = 0;
      tz = 0;
      return;
    case 1:
      tx = 0;
      ty = dy;
      tz = 0;
      return;
    case 2:
      tx = 0;
      ty = 0;
      tz = dz;
      return;
    case 3:
      tx = dx;
      ty = dy;
      tz = 0;
      return;
    case 4:
      tx = dx;
      ty = 0;
      tz = dz;
      return;
    case 5:
      tx = 0;
      ty = dy;
      tz = dz;
      return;
    case 6:
      tx = dx;
      ty = dy;
      tz = dz;
      return;
    }
  }
}

void JPS3DNeib::FNeib(int dx, int dy, int dz, int norm1, int dev,
                      int &fx, int &fy, int &fz,
                      int &nx, int &ny, int &nz)
{
  switch (norm1)
  {
  case 1:
    switch (dev)
    {
    case 0:
      fx = 0;
      fy = 1;
      fz = 0;
      break;
    case 1:
      fx = 0;
      fy = -1;
      fz = 0;
      break;
    case 2:
      fx = 1;
      fy = 0;
      fz = 0;
      break;
    case 3:
      fx = 1;
      fy = 1;
      fz = 0;
      break;
    case 4:
      fx = 1;
      fy = -1;
      fz = 0;
      break;
    case 5:
      fx = -1;
      fy = 0;
      fz = 0;
      break;
    case 6:
      fx = -1;
      fy = 1;
      fz = 0;
      break;
    case 7:
      fx = -1;
      fy = -1;
      fz = 0;
      break;
    }
    nx = fx;
    ny = fy;
    nz = dz;
    // switch order if different direction
    if (dx != 0)
    {
      fz = fx;
      fx = 0;
      nz = fz;
      nx = dx;
    }
    if (dy != 0)
    {
      fz = fy;
      fy = 0;
      nz = fz;
      ny = dy;
    }
    return;
  case 2:
    if (dx == 0)
    {
      switch (dev)
      {
      case 0:
        fx = 0;
        fy = 0;
        fz = -dz;
        nx = 0;
        ny = dy;
        nz = -dz;
        return;
      case 1:
        fx = 0;
        fy = -dy;
        fz = 0;
        nx = 0;
        ny = -dy;
        nz = dz;
        return;
      case 2:
        fx = 1;
        fy = 0;
        fz = 0;
        nx = 1;
        ny = dy;
        nz = dz;
        return;
      case 3:
        fx = -1;
        fy = 0;
        fz = 0;
        nx = -1;
        ny = dy;
        nz = dz;
        return;
      case 4:
        fx = 1;
        fy = 0;
        fz = -dz;
        nx = 1;
        ny = dy;
        nz = -dz;
        return;
      case 5:
        fx = 1;
        fy = -dy;
        fz = 0;
        nx = 1;
        ny = -dy;
        nz = dz;
        return;
      case 6:
        fx = -1;
        fy = 0;
        fz = -dz;
        nx = -1;
        ny = dy;
        nz = -dz;
        return;
      case 7:
        fx = -1;
        fy = -dy;
        fz = 0;
        nx = -1;
        ny = -dy;
        nz = dz;
        return;
      // Extras
      case 8:
        fx = 1;
        fy = 0;
        fz = 0;
        nx = 1;
        ny = dy;
        nz = 0;
        return;
      case 9:
        fx = 1;
        fy = 0;
        fz = 0;
        nx = 1;
        ny = 0;
        nz = dz;
        return;
      case 10:
        fx = -1;
        fy = 0;
        fz = 0;
        nx = -1;
        ny = dy;
        nz = 0;
        return;
      case 11:
        fx = -1;
        fy = 0;
        fz = 0;
        nx = -1;
        ny = 0;
        nz = dz;
        return;
      }
    }
    else if (dy == 0)
    {
      switch (dev)
      {
      case 0:
        fx = 0;
        fy = 0;
        fz = -dz;
        nx = dx;
        ny = 0;
        nz = -dz;
        return;
      case 1:
        fx = -dx;
        fy = 0;
        fz = 0;
        nx = -dx;
        ny = 0;
        nz = dz;
        return;
      case 2:
        fx = 0;
        fy = 1;
        fz = 0;
        nx = dx;
        ny = 1;
        nz = dz;
        return;
      case 3:
        fx = 0;
        fy = -1;
        fz = 0;
        nx = dx;
        ny = -1;
        nz = dz;
        return;
      case 4:
        fx = 0;
        fy = 1;
        fz = -dz;
        nx = dx;
        ny = 1;
        nz = -dz;
        return;
      case 5:
        fx = -dx;
        fy = 1;
        fz = 0;
        nx = -dx;
        ny = 1;
        nz = dz;
        return;
      case 6:
        fx = 0;
        fy = -1;
        fz = -dz;
        nx = dx;
        ny = -1;
        nz = -dz;
        return;
      case 7:
        fx = -dx;
        fy = -1;
        fz = 0;
        nx = -dx;
        ny = -1;
        nz = dz;
        return;
      // Extras
      case 8:
        fx = 0;
        fy = 1;
        fz = 0;
        nx = dx;
        ny = 1;
        nz = 0;
        return;
      case 9:
        fx = 0;
        fy = 1;
        fz = 0;
        nx = 0;
        ny = 1;
        nz = dz;
        return;
      case 10:
        fx = 0;
        fy = -1;
        fz = 0;
        nx = dx;
        ny = -1;
        nz = 0;
        return;
      case 11:
        fx = 0;
        fy = -1;
        fz = 0;
        nx = 0;
        ny = -1;
        nz = dz;
        return;
      }
    }
    else
    { // dz==0
      switch (dev)
      {
      case 0:
        fx = 0;
        fy = -dy;
        fz = 0;
        nx = dx;
        ny = -dy;
        nz = 0;
        return;
      case 1:
        fx = -dx;
        fy = 0;
        fz = 0;
        nx = -dx;
        ny = dy;
        nz = 0;
        return;
      case 2:
        fx = 0;
        fy = 0;
        fz = 1;
        nx = dx;
        ny = dy;
        nz = 1;
        return;
      case 3:
        fx = 0;
        fy = 0;
        fz = -1;
        nx = dx;
        ny = dy;
        nz = -1;
        return;
      case 4:
        fx = 0;
        fy = -dy;
        fz = 1;
        nx = dx;
        ny = -dy;
        nz = 1;
        return;
      case 5:
        fx = -dx;
        fy = 0;
        fz = 1;
        nx = -dx;
        ny = dy;
        nz = 1;
        return;
      case 6:
        fx = 0;
        fy = -dy;
        fz = -1;
        nx = dx;
        ny = -dy;
        nz = -1;
        return;
      case 7:
        fx = -dx;
        fy = 0;
        fz = -1;
        nx = -dx;
        ny = dy;
        nz = -1;
        return;
      // Extras
      case 8:
        fx = 0;
        fy = 0;
        fz = 1;
        nx = dx;
        ny = 0;
        nz = 1;
        return;
      case 9:
        fx = 0;
        fy = 0;
        fz = 1;
        nx = 0;
        ny = dy;
        nz = 1;
        return;
      case 10:
        fx = 0;
        fy = 0;
        fz = -1;
        nx = dx;
        ny = 0;
        nz = -1;
        return;
      case 11:
        fx = 0;
        fy = 0;
        fz = -1;
        nx = 0;
        ny = dy;
        nz = -1;
        return;
      }
    }
  case 3:
    switch (dev)
    {
    case 0:
      fx = -dx;
      fy = 0;
      fz = 0;
      nx = -dx;
      ny = dy;
      nz = dz;
      return;
    case 1:
      fx = 0;
      fy = -dy;
      fz = 0;
      nx = dx;
      ny = -dy;
      nz = dz;
      return;
    case 2:
      fx = 0;
      fy = 0;
      fz = -dz;
      nx = dx;
      ny = dy;
      nz = -dz;
      return;
    // Need to check up to here for forced!
    case 3:
      fx = 0;
      fy = -dy;
      fz = -dz;
      nx = dx;
      ny = -dy;
      nz = -dz;
      return;
    case 4:
      fx = -dx;
      fy = 0;
      fz = -dz;
      nx = -dx;
      ny = dy;
      nz = -dz;
      return;
    case 5:
      fx = -dx;
      fy = -dy;
      fz = 0;
      nx = -dx;
      ny = -dy;
      nz = dz;
      return;
    // Extras
    case 6:
      fx = -dx;
      fy = 0;
      fz = 0;
      nx = -dx;
      ny = 0;
      nz = dz;
      return;
    case 7:
      fx = -dx;
      fy = 0;
      fz = 0;
      nx = -dx;
      ny = dy;
      nz = 0;
      return;
    case 8:
      fx = 0;
      fy = -dy;
      fz = 0;
      nx = 0;
      ny = -dy;
      nz = dz;
      return;
    case 9:
      fx = 0;
      fy = -dy;
      fz = 0;
      nx = dx;
      ny = -dy;
      nz = 0;
      return;
    case 10:
      fx = 0;
      fy = 0;
      fz = -dz;
      nx = 0;
      ny = dy;
      nz = -dz;
      return;
    case 11:
      fx = 0;
      fy = 0;
      fz = -dz;
      nx = dx;
      ny = 0;
      nz = -dz;
      return;
    }
  }
}
