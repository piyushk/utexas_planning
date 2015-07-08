/**
 * \file  vi_example.cpp
 * \brief A simple example demonstrating how to use Value Iteration using the bwi_rl code base to solve for an agent
 *        trying to find a goal in 10x by 10x torus grid.
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 *
 * Copyright (c) 2015, UT Austin

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 * $ Id: 05/26/2015 05:00:28 PM piyushk $
 *
 **/

#include <boost/serialization/export.hpp>

#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/examples/grid_model.h>

#include <utexas_planning/planners/vi/vi.h>

using namespace utexas_planning;

int main(int argc, char **argv) {

  GenerativeModel::Ptr model(new GridModel);
  planners::VI::Ptr planner(new planners::VI);
  YAML::Node empty_params;
  std::string out_dir = "/tmp/test";
  planner->init(model, empty_params, out_dir);
  std::cout << "Computing policy..." << std::endl;

  planner->performEpisodeStartProcessing(State::ConstPtr());

  std::vector<boost::shared_ptr<GridState> > test_states;
  boost::shared_ptr<GridState> test(new GridState);
  test->x = 5;
  test->y = 2;
  test_states.push_back(test);
  test.reset(new GridState);
  test->x = 2;
  test->y = 5;
  test_states.push_back(test);
  test.reset(new GridState);
  test->x = 5;
  test->y = 8;
  test_states.push_back(test);
  test.reset(new GridState);
  test->x = 8;
  test->y = 5;
  test_states.push_back(test);

  BOOST_FOREACH(const boost::shared_ptr<GridState>& s, test_states) {
    Action::ConstPtr action_base = planner->getBestAction(s);
    std::cout << "Best action at state " << *s << " is " << *action_base << std::endl;
  }

  return 0;
}
