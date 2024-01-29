//#line 2 "/opt/ros/noetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"
// *********************************************************
//
// File autogenerated for the global_planner package
// by the dynamic_reconfigure package.
// Please do not edit.
//
// ********************************************************/

#ifndef __global_planner__GLOBALPLANNERCONFIG_H__
#define __global_planner__GLOBALPLANNERCONFIG_H__

#if __cplusplus >= 201103L
#define DYNAMIC_RECONFIGURE_FINAL final
#else
#define DYNAMIC_RECONFIGURE_FINAL
#endif

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace global_planner
{
  class GlobalPlannerConfigStatics;

  class GlobalPlannerConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l,
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      virtual ~AbstractParamDescription() = default;

      virtual void clamp(GlobalPlannerConfig &config, const GlobalPlannerConfig &max, const GlobalPlannerConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const GlobalPlannerConfig &config1, const GlobalPlannerConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, GlobalPlannerConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const GlobalPlannerConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, GlobalPlannerConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const GlobalPlannerConfig &config) const = 0;
      virtual void getValue(const GlobalPlannerConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template <class T>
    class ParamDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string a_name, std::string a_type, uint32_t a_level,
          std::string a_description, std::string a_edit_method, T GlobalPlannerConfig::* a_f) :
        AbstractParamDescription(a_name, a_type, a_level, a_description, a_edit_method),
        field(a_f)
      {}

      T GlobalPlannerConfig::* field;

      virtual void clamp(GlobalPlannerConfig &config, const GlobalPlannerConfig &max, const GlobalPlannerConfig &min) const override
      {
        if (config.*field > max.*field)
          config.*field = max.*field;

        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const GlobalPlannerConfig &config1, const GlobalPlannerConfig &config2) const override
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, GlobalPlannerConfig &config) const override
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const GlobalPlannerConfig &config) const override
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, GlobalPlannerConfig &config) const override
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const GlobalPlannerConfig &config) const override
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const GlobalPlannerConfig &config, boost::any &val) const override
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      virtual ~AbstractGroupDescription() = default;

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, GlobalPlannerConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template<class T, class PT>
    class GroupDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string a_name, std::string a_type, int a_parent, int a_id, bool a_s, T PT::* a_f) : AbstractGroupDescription(a_name, a_type, a_parent, a_id, a_s), field(a_f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, GlobalPlannerConfig &top) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const override
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T PT::* field;
      std::vector<GlobalPlannerConfig::AbstractGroupDescriptionConstPtr> groups;
    };

class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(GlobalPlannerConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("lethal_cost"==(*_i)->name){lethal_cost = boost::any_cast<int>(val);}
        if("neutral_cost"==(*_i)->name){neutral_cost = boost::any_cast<int>(val);}
        if("cost_factor"==(*_i)->name){cost_factor = boost::any_cast<double>(val);}
        if("publish_potential"==(*_i)->name){publish_potential = boost::any_cast<bool>(val);}
        if("orientation_mode"==(*_i)->name){orientation_mode = boost::any_cast<int>(val);}
        if("orientation_window_size"==(*_i)->name){orientation_window_size = boost::any_cast<int>(val);}
      }
    }

    int lethal_cost;
int neutral_cost;
double cost_factor;
bool publish_potential;
int orientation_mode;
int orientation_window_size;

    bool state;
    std::string name;

    
}groups;



//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int lethal_cost;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int neutral_cost;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double cost_factor;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool publish_potential;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int orientation_mode;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int orientation_window_size;
//#line 231 "/opt/ros/noetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("GlobalPlannerConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }

    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }

    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const GlobalPlannerConfig &__max__ = __getMax__();
      const GlobalPlannerConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const GlobalPlannerConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }

    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const GlobalPlannerConfig &__getDefault__();
    static const GlobalPlannerConfig &__getMax__();
    static const GlobalPlannerConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

  private:
    static const GlobalPlannerConfigStatics *__get_statics__();
  };

  template <> // Max and min are ignored for strings.
  inline void GlobalPlannerConfig::ParamDescription<std::string>::clamp(GlobalPlannerConfig &config, const GlobalPlannerConfig &max, const GlobalPlannerConfig &min) const
  {
    (void) config;
    (void) min;
    (void) max;
    return;
  }

  class GlobalPlannerConfigStatics
  {
    friend class GlobalPlannerConfig;

    GlobalPlannerConfigStatics()
    {
GlobalPlannerConfig::GroupDescription<GlobalPlannerConfig::DEFAULT, GlobalPlannerConfig> Default("Default", "", 0, 0, true, &GlobalPlannerConfig::groups);
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.lethal_cost = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.lethal_cost = 255;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.lethal_cost = 253;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(GlobalPlannerConfig::AbstractParamDescriptionConstPtr(new GlobalPlannerConfig::ParamDescription<int>("lethal_cost", "int", 0, "Lethal Cost", "", &GlobalPlannerConfig::lethal_cost)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(GlobalPlannerConfig::AbstractParamDescriptionConstPtr(new GlobalPlannerConfig::ParamDescription<int>("lethal_cost", "int", 0, "Lethal Cost", "", &GlobalPlannerConfig::lethal_cost)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.neutral_cost = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.neutral_cost = 255;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.neutral_cost = 50;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(GlobalPlannerConfig::AbstractParamDescriptionConstPtr(new GlobalPlannerConfig::ParamDescription<int>("neutral_cost", "int", 0, "Neutral Cost", "", &GlobalPlannerConfig::neutral_cost)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(GlobalPlannerConfig::AbstractParamDescriptionConstPtr(new GlobalPlannerConfig::ParamDescription<int>("neutral_cost", "int", 0, "Neutral Cost", "", &GlobalPlannerConfig::neutral_cost)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.cost_factor = 0.01;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.cost_factor = 5.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.cost_factor = 3.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(GlobalPlannerConfig::AbstractParamDescriptionConstPtr(new GlobalPlannerConfig::ParamDescription<double>("cost_factor", "double", 0, "Factor to multiply each cost from costmap by", "", &GlobalPlannerConfig::cost_factor)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(GlobalPlannerConfig::AbstractParamDescriptionConstPtr(new GlobalPlannerConfig::ParamDescription<double>("cost_factor", "double", 0, "Factor to multiply each cost from costmap by", "", &GlobalPlannerConfig::cost_factor)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.publish_potential = 0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.publish_potential = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.publish_potential = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(GlobalPlannerConfig::AbstractParamDescriptionConstPtr(new GlobalPlannerConfig::ParamDescription<bool>("publish_potential", "bool", 0, "Publish Potential Costmap", "", &GlobalPlannerConfig::publish_potential)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(GlobalPlannerConfig::AbstractParamDescriptionConstPtr(new GlobalPlannerConfig::ParamDescription<bool>("publish_potential", "bool", 0, "Publish Potential Costmap", "", &GlobalPlannerConfig::publish_potential)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.orientation_mode = 0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.orientation_mode = 6;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.orientation_mode = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(GlobalPlannerConfig::AbstractParamDescriptionConstPtr(new GlobalPlannerConfig::ParamDescription<int>("orientation_mode", "int", 0, "How to set the orientation of each point", "{'enum': [{'name': 'None', 'type': 'int', 'value': 0, 'srcline': 14, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'No orientations added except goal orientation', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Forward', 'type': 'int', 'value': 1, 'srcline': 15, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'Positive x axis points along path, except for the goal orientation', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Interpolate', 'type': 'int', 'value': 2, 'srcline': 17, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'Orientations are a linear blend of start and goal pose', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'ForwardThenInterpolate', 'type': 'int', 'value': 3, 'srcline': 18, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'Forward orientation until last straightaway, then a linear blend until the goal pose', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Backward', 'type': 'int', 'value': 4, 'srcline': 20, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'Negative x axis points along the path, except for the goal orientation', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Leftward', 'type': 'int', 'value': 5, 'srcline': 22, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'Positive y axis points along the path, except for the goal orientation', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Rightward', 'type': 'int', 'value': 6, 'srcline': 24, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'Negative y axis points along the path, except for the goal orientation', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'How to set the orientation of each point'}", &GlobalPlannerConfig::orientation_mode)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(GlobalPlannerConfig::AbstractParamDescriptionConstPtr(new GlobalPlannerConfig::ParamDescription<int>("orientation_mode", "int", 0, "How to set the orientation of each point", "{'enum': [{'name': 'None', 'type': 'int', 'value': 0, 'srcline': 14, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'No orientations added except goal orientation', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Forward', 'type': 'int', 'value': 1, 'srcline': 15, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'Positive x axis points along path, except for the goal orientation', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Interpolate', 'type': 'int', 'value': 2, 'srcline': 17, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'Orientations are a linear blend of start and goal pose', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'ForwardThenInterpolate', 'type': 'int', 'value': 3, 'srcline': 18, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'Forward orientation until last straightaway, then a linear blend until the goal pose', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Backward', 'type': 'int', 'value': 4, 'srcline': 20, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'Negative x axis points along the path, except for the goal orientation', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Leftward', 'type': 'int', 'value': 5, 'srcline': 22, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'Positive y axis points along the path, except for the goal orientation', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Rightward', 'type': 'int', 'value': 6, 'srcline': 24, 'srcfile': '/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg', 'description': 'Negative y axis points along the path, except for the goal orientation', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'How to set the orientation of each point'}", &GlobalPlannerConfig::orientation_mode)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.orientation_window_size = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.orientation_window_size = 255;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.orientation_window_size = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(GlobalPlannerConfig::AbstractParamDescriptionConstPtr(new GlobalPlannerConfig::ParamDescription<int>("orientation_window_size", "int", 0, "What window to use to determine the orientation based on the position derivative specified by the orientation mode", "", &GlobalPlannerConfig::orientation_window_size)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(GlobalPlannerConfig::AbstractParamDescriptionConstPtr(new GlobalPlannerConfig::ParamDescription<int>("orientation_window_size", "int", 0, "What window to use to determine the orientation based on the position derivative specified by the orientation mode", "", &GlobalPlannerConfig::orientation_window_size)));
//#line 246 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.convertParams();
//#line 246 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(GlobalPlannerConfig::AbstractGroupDescriptionConstPtr(new GlobalPlannerConfig::GroupDescription<GlobalPlannerConfig::DEFAULT, GlobalPlannerConfig>(Default)));
//#line 369 "/opt/ros/noetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

      for (std::vector<GlobalPlannerConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__);
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__);
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__);
    }
    std::vector<GlobalPlannerConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<GlobalPlannerConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    GlobalPlannerConfig __max__;
    GlobalPlannerConfig __min__;
    GlobalPlannerConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const GlobalPlannerConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static GlobalPlannerConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &GlobalPlannerConfig::__getDescriptionMessage__()
  {
    return __get_statics__()->__description_message__;
  }

  inline const GlobalPlannerConfig &GlobalPlannerConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }

  inline const GlobalPlannerConfig &GlobalPlannerConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }

  inline const GlobalPlannerConfig &GlobalPlannerConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }

  inline const std::vector<GlobalPlannerConfig::AbstractParamDescriptionConstPtr> &GlobalPlannerConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<GlobalPlannerConfig::AbstractGroupDescriptionConstPtr> &GlobalPlannerConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const GlobalPlannerConfigStatics *GlobalPlannerConfig::__get_statics__()
  {
    const static GlobalPlannerConfigStatics *statics;

    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = GlobalPlannerConfigStatics::get_instance();

    return statics;
  }

//#line 14 "/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg"
      const int GlobalPlanner_None = 0;
//#line 15 "/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg"
      const int GlobalPlanner_Forward = 1;
//#line 17 "/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg"
      const int GlobalPlanner_Interpolate = 2;
//#line 18 "/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg"
      const int GlobalPlanner_ForwardThenInterpolate = 3;
//#line 20 "/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg"
      const int GlobalPlanner_Backward = 4;
//#line 22 "/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg"
      const int GlobalPlanner_Leftward = 5;
//#line 24 "/home/twang/Turtlebot3_AutoRace/src/navigation/global_planner/cfg/GlobalPlanner.cfg"
      const int GlobalPlanner_Rightward = 6;
}

#undef DYNAMIC_RECONFIGURE_FINAL

#endif // __GLOBALPLANNERRECONFIGURATOR_H__
