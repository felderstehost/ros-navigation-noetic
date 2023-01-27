/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
//
// Navigation function computation
// Uses Dijkstra's method
// Modified for Euclidean-distance computation
//
// Path calculation uses no interpolation when pot field is at max in
//   nearby cells
//
// Path calc has sanity check that it succeeded
//


#include <navfn/navfn.h>
#include <ros/console.h>

namespace navfn {

  //
  // function to perform nav fn calculation
  // keeps track of internal buffers, will be more efficient
  //   if the size of the environment does not change
  //

  int
    create_nav_plan_astar(COSTTYPE *costmap, int nx, int ny,
        int* goal, int* start,
        float *plan, int nplan)
    {
      static NavFn *nav = NULL;

      if (nav == NULL)
        nav = new NavFn(nx,ny);

      if (nav->nx != nx || nav->ny != ny) // check for compatibility with previous call
      {
        delete nav;
        nav = new NavFn(nx,ny);
      }

      nav->setGoal(goal);
      nav->setStart(start);

      nav->costarr = costmap;
      nav->setupNavFn(true);

      // calculate the nav fn and path
      nav->priInc = 2*COST_NEUTRAL;
      nav->propNavFnAstar(std::max(nx*ny/20,nx+ny));

      // path
      int len = nav->calcPath(nplan);

      if (len > 0)			// found plan
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
      else
        ROS_DEBUG("[NavFn] No path found\n");

      if (len > 0)
      {
        for (int i=0; i<len; i++)
        {
          plan[i*2] = nav->pathx[i];
          plan[i*2+1] = nav->pathy[i];
        }
      }

      return len;
    }




  //
  // create nav fn buffers
  //

  NavFn::NavFn(int xs, int ys)
  {
    // create cell arrays
    costarr = NULL;
    potarr = NULL;
    pending = NULL;
    gradx = grady = NULL;
    setNavArr(xs,ys);

    // priority buffers
    pb1 = new int[PRIORITYBUFSIZE];
    pb2 = new int[PRIORITYBUFSIZE];
    pb3 = new int[PRIORITYBUFSIZE];

    // for Dijkstra (breadth-first), set to COST_NEUTRAL
    // for A* (best-first), set to COST_NEUTRAL
    priInc = 2*COST_NEUTRAL;

    // goal and start
    goal[0] = goal[1] = 0;
    start[0] = start[1] = 0;

    // display function
    displayFn = NULL;
    displayInt = 0;

    // path buffers
    npathbuf = npath = 0;
    pathx = pathy = NULL;
    pathStep = 0.5;
  }


  NavFn::~NavFn()
  {
    if(costarr)
      delete[] costarr;
    if(potarr)
      delete[] potarr;
    if(pending)
      delete[] pending;
    if(gradx)
      delete[] gradx;
    if(grady)
      delete[] grady;
    if(pathx)
      delete[] pathx;
    if(pathy)
      delete[] pathy;
    if(pb1)
      delete[] pb1;
    if(pb2)
      delete[] pb2;
    if(pb3)
      delete[] pb3;
  }


  //
  // set goal, start positions for the nav fn
  //

  void
    NavFn::setGoal(int *g)
    {
      goal[0] = g[0];
      goal[1] = g[1];
      ROS_DEBUG("[NavFn] Setting goal to %d,%d\n", goal[0], goal[1]);
    }

  void
    NavFn::setStart(int *g)
    {
      start[0] = g[0];
      start[1] = g[1];
      ROS_DEBUG("[NavFn] Setting start to %d,%d\n", start[0], start[1]);
    }

  //
  // Set/Reset map size
  //

  void
    NavFn::setNavArr(int xs, int ys)
    {
      ROS_DEBUG("[NavFn] Array is %d x %d\n", xs, ys);

      nx = xs;
      ny = ys;
      ns = nx*ny;

      if(costarr)
        delete[] costarr;
      if(potarr)
        delete[] potarr;
      if(pending)
        delete[] pending;

      if(gradx)
        delete[] gradx;
      if(grady)
        delete[] grady;

      costarr = new COSTTYPE[ns]; // cost array, 2d config space
      memset(costarr, 0, ns*sizeof(COSTTYPE));
      potarr = new float[ns];	// navigation potential array
      pending = new bool[ns];
      memset(pending, 0, ns*sizeof(bool));
      gradx = new float[ns];
      grady = new float[ns];
    }


  //
  // set up cost array, usually from ROS
  //

  void
    NavFn::setCostmap(const COSTTYPE *cmap, bool isROS, bool allow_unknown)
    {
      // 全局规划用到的地图costarr
      COSTTYPE *cm = costarr;
      if (isROS)			// ROS类型的代价矩阵
      {
        for (int i=0; i<ny; i++)
        {
          // k值记录二重迭代的次数
          int k=i*nx;
          // cmap指向costmap元素，cm指向costarr
          for (int j=0; j<nx; j++, k++, cmap++, cm++)
          {
            // 在这里会转换输入的代价值:
            // 若当前cell在costmap上的值 == COST_OBS(254)，即致命障碍物（障碍物本身），值仍为254
            // 若当前cell在costmap上的值 == COST_UNKNOWN_ROS(255)，即未知区域，赋值为253；
            // 若当前cell在costmap上的值 < COST_OBS_ROS(253)，重新将其赋值为COST_NEUTRAL(50)+当前cell在costmap上的值×0.8，最高253

            //默认最小权重值为COST_NEUTRAL＝50，最大权重值为COST_OBS＝254
            //注：最小权重值即行走单个free(无障碍物影响)栅格所付出的权重代价
            //最大权重值即行走单个障碍物栅格所付出的权重代价

            *cm = COST_OBS;
            int v = *cmap;
            // 若当前cell的代价小于障碍物类型(253)，实际上253是膨胀型障碍
            if (v < COST_OBS_ROS)
            {
              //重新将其赋值为50+cost地图上的障碍物值×比例0.8
              v = COST_NEUTRAL+COST_FACTOR*v;
              //若值>=COST_OBS(254，致命层障碍)
              if (v >= COST_OBS)
                // 统一设置为253，确保不要超出范围
                v = COST_OBS-1;
              // 赋值给当前全局规划要使用的地图costarr
              *cm = v;
            }
            // 若当前cell的值为COST_UNKNOWN_ROS(255)，未知区域
            else if(v == COST_UNKNOWN_ROS && allow_unknown)
            {
              // 统一设置为253
              v = COST_OBS-1;
              *cm = v;
            }
          }
        }
      }

      else				// 如果地图是PGM类型，做同样的转换工作，设置costarr数组。
      {
        for (int i=0; i<ny; i++)
        {
          int k=i*nx;
          for (int j=0; j<nx; j++, k++, cmap++, cm++)
          {
            *cm = COST_OBS;
            if (i<7 || i > ny-8 || j<7 || j > nx-8)
              continue;	// 避免处理边界单元格
            int v = *cmap;
            if (v < COST_OBS_ROS)
            {
              v = COST_NEUTRAL+COST_FACTOR*v;
              if (v >= COST_OBS)
                v = COST_OBS-1;
              *cm = v;
            }
            else if(v == COST_UNKNOWN_ROS)
            {
              v = COST_OBS-1;
              *cm = v;
            }
          }
        }

      }
    }

  bool
    NavFn::calcNavFnDijkstra(bool atStart)
    {
      //重新设置势场矩阵potarr的值、设置costarr的边际值、设置目标在costarr中的值为0，对四周cell进行处理，记录costarr中障碍物cell数
      setupNavFn(true);

      // 计算navfn和全局路径
      propNavFnDijkstra(std::max(nx*ny/20,nx+ny),atStart);

      // 全局路径
      int len = calcPath(nx*ny/2);

      if (len > 0)			// 找到全局路径
      {
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
        return true;
      }
      else
      {
        ROS_DEBUG("[NavFn] No path found\n");
        return false;
      }

    }


  // 根据代价地图，目标点和起始点计算navigation function
  bool
    NavFn::calcNavFnAstar()
    {
      setupNavFn(true);

      // 计算  nav fn 和全局路径
      propNavFnAstar(std::max(nx*ny/20,nx+ny));

      // 路径
      int len = calcPath(nx*4);

      if (len > 0)			// 计算出有效全局路径
      {
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
        return true;
      }
      else
      {
        ROS_DEBUG("[NavFn] No path found\n");
        return false;
      }
    }


  //
  // returning values
  //

  float *NavFn::getPathX() { return pathx; }
  float *NavFn::getPathY() { return pathy; }
  int    NavFn::getPathLen() { return npath; }

  // 插入到 the priority blocks
#define push_cur(n)  { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && curPe<PRIORITYBUFSIZE) \
  { curP[curPe++]=n; pending[n]=true; }}
#define push_next(n) { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && nextPe<PRIORITYBUFSIZE) \
  { nextP[nextPe++]=n; pending[n]=true; }}
#define push_over(n) { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && overPe<PRIORITYBUFSIZE) \
  { overP[overPe++]=n; pending[n]=true; }}


  // Set up navigation potential arrays for new propagation
  // 该函数对转换的costarr数组进行了边际设置等处理，并初始化了potarr数组和梯度数组gradx、grady。
  // 下面先循环初始化potarr矩阵元素全部为最大值POT_HIGH，并初始化梯度表初始值全部为0.0
  void
    NavFn::setupNavFn(bool keepit)
    {
      // step 1: 重新设置potarr的值
      for (int i=0; i<ns; i++)
      {
        // 将potarr初始化为最大值，默认起点到所有点的行走代价值都为最大
        potarr[i] = POT_HIGH;
        if (!keepit) costarr[i] = COST_NEUTRAL;
        // 初始化x,y方向的梯度表
        gradx[i] = grady[i] = 0.0;
      }

      // step 2: 接下来设置costarr的四条边的cell的值为COST_OBS(致命层254)，封闭地图四周，以防产生边界以外的轨迹。
      COSTTYPE *pc;
      pc = costarr;
      // costarr第一行全部设置为COST_OBS(致命层254)
      for (int i=0; i<nx; i++)
        *pc++ = COST_OBS;
      // costarr最后一行全部设置为COST_OBS(致命层254)
      pc = costarr + (ny-1)*nx;
      for (int i=0; i<nx; i++)
        *pc++ = COST_OBS;
      pc = costarr;
      // costarr第一列全部设置为COST_OBS(致命层254)
      for (int i=0; i<ny; i++, pc+=nx)
        *pc = COST_OBS;
      pc = costarr + nx - 1;
      // costarr最后一列全部设置为COST_OBS(致命层254)
      for (int i=0; i<ny; i++, pc+=nx)
        *pc = COST_OBS;

      // step 3: 初始化一些用于迭代更新potarr的数据，并初始化pending数组为全0，设置所有的cell状态都为非等待状态。
      // 优先级缓冲
      curT = COST_OBS; // 传播阈值设为254
      curP = pb1; // 当前用于传播的cell索引数组
      curPe = 0; //当前用于传播的cell的数量
      nextP = pb2; //用于下个传播过程的cell索引数组
      nextPe = 0; //用于下个传播过程的cell的数量
      overP = pb3; //传播界限外的cell索引数组
      overPe = 0; //传播界限外的cell的数量
      memset(pending, 0, ns*sizeof(bool));

      // k为目标cell的索引
      int k = goal[0] + goal[1]*nx;
      // step 4: costarr的索引k（目标点）的元素值设为0，并对它四周的cell在pending[]中进行标记“等待状态”，并把索引存放入curP数组
      initCost(k,0);

      // step 5: 更新障碍物单元格的数量nobs
      pc = costarr;
      int ntot = 0;
      for (int i=0; i<ns; i++, pc++)
      {
        if (*pc >= COST_OBS)
          ntot++;			// 记录costarr中的致命障碍物cell的数量
      }
      nobs = ntot;
    }


  // initialize a goal-type cost for starting propagation

  void
    NavFn::initCost(int k, float v)
    {
      potarr[k] = v;
      push_cur(k+1);
      push_cur(k-1);
      push_cur(k-nx);
      push_cur(k+nx);
    }


  //
  // Critical function: calculate updated potential value of a cell,
  //   given its neighbors' values
  // Planar-wave update calculation from two lowest neighbors in a 4-grid
  // Quadratic approximation to the interpolated value
  // No checking of bounds here, this function should be fast
  //

#define INVSQRT2 0.707106781

  // updateCell用于更新某个单元格(cell) 的Potential值，先获取当前cell四周邻点的potential值，并取最小的值存入ta。
  inline void
    NavFn::updateCell(int n)
    {
      // 获得邻点的potential
      float u,d,l,r;
      l = potarr[n-1];
      r = potarr[n+1];
      u = potarr[n-nx];
      d = potarr[n+nx];
      //  ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
      //	 potarr[n], l, r, u, d);
      //  ROS_INFO("[Update] cost: %d\n", costarr[n]);

      // 找到左右最小potential的单元格 ta, 上下最小potential的相邻单元格 tc
      float ta, tc;
      if (l<r) tc=l; else tc=r;
      if (u<d) ta=u; else ta=d;

      // 如果当前单元格不是致命障碍物，则四周传播，否则到它后停止，不传播。
      if (costarr[n] < COST_OBS)	// 不要传播到了障碍物那边了
      {
        float hf = (float)costarr[n]; // traversability factor 描述可通行性的因子
        float dc = tc-ta;		// ta 和 tc 的代价值差
        if (dc < 0)
        {
          dc = -dc;
          ta = tc;  // 设置ta为上下左右cell中最小的
        }

        // 只有当前cell的Potential计算值<原本的Potential值，才更新，这意味着从目标点开始，
        // 它的Potential值被初始化为0，不会被更新，接下来传播到它的四个邻点，才会开始更新他们的Potential值。
        // !checks to see if the relative cost of traversing through a cell is less than the cost of going around it
        float pot;
        if (dc >= hf)		// ta 和 tc 的代价值差如果太大, use ta-only update
          pot = ta+hf;
          
        // cost of going around a cell is less than going through it,
        else			// two-neighbor interpolation update
        {
          // use quadratic approximation
          float d = dc/hf;
          //? 这个系数怎么取的,在d=1.25时候，v取极大值1.003. 0.7 < v < 1.0
          float v = -0.2301*d*d + 0.5307*d + 0.7040; 
          pot = ta + hf*v;
        }

        //      ROS_INFO("[Update] new pot: %d\n", costarr[n]);

        // now add affected neighbors to priority blocks 当前单元格的potential比之前的小，则更新
        if (pot < potarr[n])
        {
          float le = INVSQRT2*(float)costarr[n-1];
          float re = INVSQRT2*(float)costarr[n+1];
          float ue = INVSQRT2*(float)costarr[n-nx];
          float de = INVSQRT2*(float)costarr[n+nx];
          potarr[n] = pot;
          if (pot < curT)	// 如果当前单元格potential小于curT,放入代价的缓冲区域
          {
            if (l > pot+le) push_next(n-1);
            if (r > pot+re) push_next(n+1);
            if (u > pot+ue) push_next(n-nx);
            if (d > pot+de) push_next(n+nx);
          }
          else			// 如果当前单元格potential大于curT，放入overflow block
          {
            if (l > pot+le) push_over(n-1);
            if (r > pot+re) push_over(n+1);
            if (u > pot+ue) push_over(n-nx);
            if (d > pot+de) push_over(n+nx);
          }
        }

      }

    }


  //
  // Use A* method for setting priorities
  // Critical function: calculate updated potential value of a cell,
  //   given its neighbors' values
  // Planar-wave update calculation from two lowest neighbors in a 4-grid
  // Quadratic approximation to the interpolated value
  // No checking of bounds here, this function should be fast
  //

#define INVSQRT2 0.707106781

  inline void
    NavFn::updateCellAstar(int n)
    {
      // get neighbors
      float u,d,l,r;
      l = potarr[n-1];
      r = potarr[n+1];
      u = potarr[n-nx];
      d = potarr[n+nx];
      //ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
      //	 potarr[n], l, r, u, d);
      // ROS_INFO("[Update] cost of %d: %d\n", n, costarr[n]);

      // find lowest, and its lowest neighbor
      float ta, tc;
      if (l<r) tc=l; else tc=r;
      if (u<d) ta=u; else ta=d;

      // do planar wave update
      if (costarr[n] < COST_OBS)	// don't propagate into obstacles
      {
        float hf = (float)costarr[n]; // traversability factor
        float dc = tc-ta;		// relative cost between ta,tc
        if (dc < 0) 		// ta is lowest
        {
          dc = -dc;
          ta = tc;
        }

        // calculate new potential
        float pot;
        if (dc >= hf)		// if too large, use ta-only update
          pot = ta+hf;
        else			// two-neighbor interpolation update
        {
          // use quadratic approximation
          // might speed this up through table lookup, but still have to
          //   do the divide
          float d = dc/hf;
          float v = -0.2301*d*d + 0.5307*d + 0.7040;
          pot = ta + hf*v;
        }

        //ROS_INFO("[Update] new pot: %d\n", costarr[n]);

        // now add affected neighbors to priority blocks
        if (pot < potarr[n])
        {
          float le = INVSQRT2*(float)costarr[n-1];
          float re = INVSQRT2*(float)costarr[n+1];
          float ue = INVSQRT2*(float)costarr[n-nx];
          float de = INVSQRT2*(float)costarr[n+nx];

          // calculate distance
          int x = n%nx;
          int y = n/nx;
          float dist = hypot(x-start[0], y-start[1])*(float)COST_NEUTRAL;

          potarr[n] = pot;
          pot += dist;
          if (pot < curT)	// low-cost buffer block
          {
            if (l > pot+le) push_next(n-1);
            if (r > pot+re) push_next(n+1);
            if (u > pot+ue) push_next(n-nx);
            if (d > pot+de) push_next(n+nx);
          }
          else
          {
            if (l > pot+le) push_over(n-1);
            if (r > pot+re) push_over(n+1);
            if (u > pot+ue) push_over(n-nx);
            if (d > pot+de) push_over(n+nx);
          }
        }

      }

    }



  // propNavFnDijkstra是主要的传播函数，Dijkstra算法，一个广度优先算法， 运行cycles次或者直到所以的单元格被更新或者起始单元格已经被找到 (atStart = true)
  bool
    NavFn::propNavFnDijkstra(int cycles, bool atStart)
    {
      int nwv = 0;			// priority block 的最大值
      int nc = 0;			// 放到priority blocks中单元格数量
      int cycle = 0;		// which cycle we're on 当前迭代次数

      // 记录起始位置的索引
      int startCell = start[1]*nx + start[0];

      // 循环迭代更新potarr，判断条件：如果当前正在传播和下一步传播的集都为空，那么说明已经无法继续传播，可能有无法越过的障碍或其他情况，退出。
      for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
      {
        //
        if (curPe == 0 && nextPe == 0) // priority blocks 为空
          break;

        // 统计用的，不影响下面计算
        nc += curPe;
        if (curPe > nwv)
          nwv = curPe;

        // 对pending数组重置为false
        int *pb = curP;
        int i = curPe;
        while (i-- > 0)
          pending[*(pb++)] = false;

        // process current priority buffer
        pb = curP; // 把迭代到终点的pb重置回curP
        i = curPe;
        while (i-- > 0)
          updateCell(*pb++);

        if (displayInt > 0 &&  (cycle % displayInt) == 0)
          displayFn(this);

        // 交换 priority blocks，即 curP <=> nextP
        curPe = nextPe;
        nextPe = 0;

        pb = curP;		// 交换缓存
        curP = nextP;
        nextP = pb;

        // see if we're done with this priority level
        if (curPe == 0)
        {
          curT += priInc;	// increment priority threshold
          curPe = overPe;	// set current to overflow block
          overPe = 0;
          pb = curP;		// swap buffers
          curP = overP;
          overP = pb;
        }

        // 在从目标点向全地图传播的过程中检查，当起点的Potential值不再是被初始化的无穷大，而是有一个实际的值时，说明到达了起点，传播停止。
        // check if we've hit the Start cell
        if (atStart)
          if (potarr[startCell] < POT_HIGH)
            break;
      }

      ROS_DEBUG("[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
          cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);

      if (cycle < cycles) return true; // finished up here
      else return false;
    }


  //
  // main propagation function
  // A* method, best-first
  // uses Euclidean distance heuristic
  // runs for a specified number of cycles,
  //   or until it runs out of cells to update,
  //   or until the Start cell is found (atStart = true)
  //

  bool
    NavFn::propNavFnAstar(int cycles)
    {
      int nwv = 0;			// max priority block size
      int nc = 0;			// number of cells put into priority blocks
      int cycle = 0;		// which cycle we're on

      // set initial threshold, based on distance
      float dist = hypot(goal[0]-start[0], goal[1]-start[1])*(float)COST_NEUTRAL;
      curT = dist + curT;

      // set up start cell
      int startCell = start[1]*nx + start[0];

      // do main cycle
      for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
      {
        //
        if (curPe == 0 && nextPe == 0) // priority blocks empty
          break;

        // stats
        nc += curPe;
        if (curPe > nwv)
          nwv = curPe;

        // reset pending flags on current priority buffer
        int *pb = curP;
        int i = curPe;
        while (i-- > 0)
          pending[*(pb++)] = false;

        // process current priority buffer
        pb = curP;
        i = curPe;
        while (i-- > 0)
          updateCellAstar(*pb++);

        if (displayInt > 0 &&  (cycle % displayInt) == 0)
          displayFn(this);

        // swap priority blocks curP <=> nextP
        curPe = nextPe;
        nextPe = 0;
        pb = curP;		// swap buffers
        curP = nextP;
        nextP = pb;

        // see if we're done with this priority level
        if (curPe == 0)
        {
          curT += priInc;	// increment priority threshold
          curPe = overPe;	// set current to overflow block
          overPe = 0;
          pb = curP;		// swap buffers
          curP = overP;
          overP = pb;
        }

        // check if we've hit the Start cell
        if (potarr[startCell] < POT_HIGH)
          break;

      }

      last_path_cost_ = potarr[startCell];

      ROS_DEBUG("[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
          cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);


      if (potarr[startCell] < POT_HIGH) return true; // finished up here
      else return false;
    }


  float NavFn::getLastPathCost()
  {
    return last_path_cost_;
  }


  //
  // 构建全局路径
  //  找到矩阵点的梯度，插入路径，用到的步长一般是0.5个像素
  //
  // Some sanity checks:
  //  1. Stuck at same index position
  //  2. Doesn't get near goal
  //  3. Surrounded by high potentials
  //
  // 该函数负责在potarr数组的基础上选取一些cell点来生成最终的全局规划路径，从起点开始沿着最优行走代价值梯度下降的方向寻找到目标点的最优轨迹。
  int
    NavFn::calcPath(int n, int *st)
    {
      // test write
      //savemap("test");

      // check path arrays
      if (npathbuf < n)
      {
        if (pathx) delete [] pathx;
        if (pathy) delete [] pathy;
        pathx = new float[n];
        pathy = new float[n];
        npathbuf = n;
      }

      // set up start position at cell
      // st is always upper left corner for 4-point bilinear interpolation
      if (st == NULL) st = start; // st指向起点
      int stc = st[1]*nx + st[0]; // stc记录起点索引

      // 初始化补偿
      float dx=0;
      float dy=0;
      npath = 0; //路径点个数

      // 最多循环cycles次
      for (int i=0; i<n; i++)
      {
        // 检查是否离目标点很近了
        int nearest_point=std::max(0,std::min(nx*ny-1,stc+(int)round(dx)+(int)(nx*round(dy))));
        if (potarr[nearest_point] < COST_NEUTRAL)
        {
          pathx[npath] = (float)goal[0];
          pathy[npath] = (float)goal[1];
          return ++npath;	// 成功找到路径后，从这里返回
        }

        if (stc < nx || stc > ns-nx) // 在第一行或最后一行，即超出边界
        {
          ROS_DEBUG("[PathCalc] Out of bounds");
          return 0;
        }

        // 添加至路径点
        pathx[npath] = stc%nx + dx; // x方向索引
        pathy[npath] = stc/nx + dy; // y方向索引
        npath++;
        // 震荡检测，某一步和上上一步的位置是否一样
        bool oscillation_detected = false;
        if( npath > 2 &&
            pathx[npath-1] == pathx[npath-3] &&
            pathy[npath-1] == pathy[npath-3] )
        {
          ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
          oscillation_detected = true;
        }
        // 当前点下方的点的索引
        int stcnx = stc+nx;
        // 当前点上方的点的索引
        int stcpx = stc-nx;

        // 检查当前到达节点的周边的8个节点是否有障碍物代价值，如果有的话，则直接将stc指向这8个节点中potential值最低的节点
        if (potarr[stc] >= POT_HIGH ||
            potarr[stc+1] >= POT_HIGH ||
            potarr[stc-1] >= POT_HIGH ||
            potarr[stcnx] >= POT_HIGH ||
            potarr[stcnx+1] >= POT_HIGH ||
            potarr[stcnx-1] >= POT_HIGH ||
            potarr[stcpx] >= POT_HIGH ||
            potarr[stcpx+1] >= POT_HIGH ||
            potarr[stcpx-1] >= POT_HIGH ||
            oscillation_detected)
        {
          ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);
          // 找出八个邻点中的最小值
          int minc = stc;
          int minp = potarr[stc];
          int st = stcpx - 1;
          //从左上角邻点开始
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st++;
          //上方邻点
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st++;
          //右上方邻点
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st = stc-1;
          //左邻点
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st = stc+1;
          //右邻点
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st = stcnx-1;
          //左下方邻点
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st++;
          //下方邻点
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          st++;
          //右下方邻点
          if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
          stc = minc;
          dx = 0;
          dy = 0;

          ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
              potarr[stc], pathx[npath-1], pathy[npath-1]);

          if (potarr[stc] >= POT_HIGH)
          {
            ROS_DEBUG("[PathCalc] No path found, high potential");
            //savemap("navfn_highpot");
            return 0;
          }
        }

        //如果有好的梯度，则直接计算梯度，并沿着梯度方向查找下一个节点
        else
        {
          //当周围八个邻点没有障碍物
          // 计算在该点四个邻点的梯度
          gradCell(stc); //该点
          gradCell(stc+1); //该点右侧点
          gradCell(stcnx); // 该点下方点
          gradCell(stcnx+1);  // 该点右下方点


          // 得到梯度插值
          float x1 = (1.0-dx)*gradx[stc] + dx*gradx[stc+1];
          float x2 = (1.0-dx)*gradx[stcnx] + dx*gradx[stcnx+1];
          float x = (1.0-dy)*x1 + dy*x2; // 插值 x
          float y1 = (1.0-dx)*grady[stc] + dx*grady[stc+1];
          float y2 = (1.0-dx)*grady[stcnx] + dx*grady[stcnx+1];
          float y = (1.0-dy)*y1 + dy*y2; // 插值 y

          // 查看梯度
          ROS_DEBUG("[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n",
                    gradx[stc], grady[stc], gradx[stc+1], grady[stc+1],
                    gradx[stcnx], grady[stcnx], gradx[stcnx+1], grady[stcnx+1],
                    x, y);

          // 是否有零梯度，返回失败
          if (x == 0.0 && y == 0.0)
          {
            ROS_DEBUG("[PathCalc] Zero gradient");
            return 0;
          }

          // 向右边移动
          float ss = pathStep/hypot(x, y);
          dx += x*ss;
          dy += y*ss;

          // 防止溢出
          if (dx > 1.0) { stc++; dx -= 1.0; }
          if (dx < -1.0) { stc--; dx += 1.0; }
          if (dy > 1.0) { stc+=nx; dy -= 1.0; }
          if (dy < -1.0) { stc-=nx; dy += 1.0; }

        }

        //      ROS_INFO("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
        //	     potarr[stc], x, y, pathx[npath-1], pathy[npath-1]);
      }

      //  return npath;			// out of cycles, return failure
      ROS_DEBUG("[PathCalc] No path found, path too long");
      //savemap("navfn_pathlong");
      return 0;			// 超出cycles次循环 返回失败
    }


  // 给cell进行梯度计算，正值表示向右和向下
  float
    NavFn::gradCell(int n)
    {
      if (gradx[n]+grady[n] > 0.0)	// 检测这个cell
        return 1.0;

      if (n < nx || n > ns-nx)	// 超出边界
        return 0.0;

      float cv = potarr[n];
      float dx = 0.0;
      float dy = 0.0;

      // 是否有障碍物
      if (cv >= POT_HIGH)
      {
        if (potarr[n-1] < POT_HIGH)
          dx = -COST_OBS;
        else if (potarr[n+1] < POT_HIGH)
          dx = COST_OBS;

        if (potarr[n-nx] < POT_HIGH)
          dy = -COST_OBS;
        else if (potarr[n+nx] < POT_HIGH)
          dy = COST_OBS;
      }

      else				// 没有障碍物
      {
        // dx calc, average to sides
        if (potarr[n-1] < POT_HIGH)
          dx += potarr[n-1]- cv;
        if (potarr[n+1] < POT_HIGH)
          dx += cv - potarr[n+1];

        // dy calc, average to sides
        if (potarr[n-nx] < POT_HIGH)
          dy += potarr[n-nx]- cv;
        if (potarr[n+nx] < POT_HIGH)
          dy += cv - potarr[n+nx];
      }

      // 归一化
      float norm = hypot(dx, dy);
      if (norm > 0)
      {
        norm = 1.0/norm;
        gradx[n] = norm*dx;
        grady[n] = norm*dy;
      }
      return norm;
    }

  // 显示设置， n 是循环的次数， 0 代表关闭
  void
    NavFn::display(void fn(NavFn *nav), int n)
    {
      displayFn = fn;
      displayInt = n;
    }


  // debug writes 存储代价地图和起始目标点，用于debug
  void
    NavFn::savemap(const char *fname)
    {
      char fn[4096];

      ROS_DEBUG("[NavFn] Saving costmap and start/goal points");
      // write start and goal points
      sprintf(fn,"%s.txt",fname);
      FILE *fp = fopen(fn,"w");
      if (!fp)
      {
        ROS_WARN("Can't open file %s", fn);
        return;
      }
      fprintf(fp,"Goal: %d %d\nStart: %d %d\n",goal[0],goal[1],start[0],start[1]);
      fclose(fp);

      // write cost array
      if (!costarr) return;
      sprintf(fn,"%s.pgm",fname);
      fp = fopen(fn,"wb");
      if (!fp)
      {
        ROS_WARN("Can't open file %s", fn);
        return;
      }
      fprintf(fp,"P5\n%d\n%d\n%d\n", nx, ny, 0xff);
      fwrite(costarr,1,nx*ny,fp);
      fclose(fp);
    }
};
