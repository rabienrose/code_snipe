// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: visual.proto

#ifndef PROTOBUF_visual_2eproto__INCLUDED
#define PROTOBUF_visual_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2005000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "pose.pb.h"
#include "geometry.pb.h"
#include "material.pb.h"
#include "plugin.pb.h"
#include "vector3d.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_visual_2eproto();
void protobuf_AssignDesc_visual_2eproto();
void protobuf_ShutdownFile_visual_2eproto();

class Visual;
class Visual_Meta;

enum Visual_Type {
  Visual_Type_ENTITY = 0,
  Visual_Type_MODEL = 1,
  Visual_Type_LINK = 2,
  Visual_Type_VISUAL = 3,
  Visual_Type_COLLISION = 4,
  Visual_Type_SENSOR = 5,
  Visual_Type_GUI = 6,
  Visual_Type_PHYSICS = 7
};
GZ_MSGS_VISIBLE bool Visual_Type_IsValid(int value);
const Visual_Type Visual_Type_Type_MIN = Visual_Type_ENTITY;
const Visual_Type Visual_Type_Type_MAX = Visual_Type_PHYSICS;
const int Visual_Type_Type_ARRAYSIZE = Visual_Type_Type_MAX + 1;

GZ_MSGS_VISIBLE const ::google::protobuf::EnumDescriptor* Visual_Type_descriptor();
inline const ::std::string& Visual_Type_Name(Visual_Type value) {
  return ::google::protobuf::internal::NameOfEnum(
    Visual_Type_descriptor(), value);
}
inline bool Visual_Type_Parse(
    const ::std::string& name, Visual_Type* value) {
  return ::google::protobuf::internal::ParseNamedEnum<Visual_Type>(
    Visual_Type_descriptor(), name, value);
}
// ===================================================================

class GZ_MSGS_VISIBLE Visual_Meta : public ::google::protobuf::Message {
 public:
  Visual_Meta();
  virtual ~Visual_Meta();

  Visual_Meta(const Visual_Meta& from);

  inline Visual_Meta& operator=(const Visual_Meta& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Visual_Meta& default_instance();

  void Swap(Visual_Meta* other);

  // implements Message ----------------------------------------------

  Visual_Meta* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Visual_Meta& from);
  void MergeFrom(const Visual_Meta& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional int32 layer = 1;
  inline bool has_layer() const;
  inline void clear_layer();
  static const int kLayerFieldNumber = 1;
  inline ::google::protobuf::int32 layer() const;
  inline void set_layer(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Visual.Meta)
 private:
  inline void set_has_layer();
  inline void clear_has_layer();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::int32 layer_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(1 + 31) / 32];

  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_visual_2eproto();
  friend void protobuf_AssignDesc_visual_2eproto();
  friend void protobuf_ShutdownFile_visual_2eproto();

  void InitAsDefaultInstance();
  static Visual_Meta* default_instance_;
};
// -------------------------------------------------------------------

class GZ_MSGS_VISIBLE Visual : public ::google::protobuf::Message {
 public:
  Visual();
  virtual ~Visual();

  Visual(const Visual& from);

  inline Visual& operator=(const Visual& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Visual& default_instance();

  void Swap(Visual* other);

  // implements Message ----------------------------------------------

  Visual* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Visual& from);
  void MergeFrom(const Visual& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  typedef Visual_Meta Meta;

  typedef Visual_Type Type;
  static const Type ENTITY = Visual_Type_ENTITY;
  static const Type MODEL = Visual_Type_MODEL;
  static const Type LINK = Visual_Type_LINK;
  static const Type VISUAL = Visual_Type_VISUAL;
  static const Type COLLISION = Visual_Type_COLLISION;
  static const Type SENSOR = Visual_Type_SENSOR;
  static const Type GUI = Visual_Type_GUI;
  static const Type PHYSICS = Visual_Type_PHYSICS;
  static inline bool Type_IsValid(int value) {
    return Visual_Type_IsValid(value);
  }
  static const Type Type_MIN =
    Visual_Type_Type_MIN;
  static const Type Type_MAX =
    Visual_Type_Type_MAX;
  static const int Type_ARRAYSIZE =
    Visual_Type_Type_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  Type_descriptor() {
    return Visual_Type_descriptor();
  }
  static inline const ::std::string& Type_Name(Type value) {
    return Visual_Type_Name(value);
  }
  static inline bool Type_Parse(const ::std::string& name,
      Type* value) {
    return Visual_Type_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // required string name = 1;
  inline bool has_name() const;
  inline void clear_name();
  static const int kNameFieldNumber = 1;
  inline const ::std::string& name() const;
  inline void set_name(const ::std::string& value);
  inline void set_name(const char* value);
  inline void set_name(const char* value, size_t size);
  inline ::std::string* mutable_name();
  inline ::std::string* release_name();
  inline void set_allocated_name(::std::string* name);

  // optional uint32 id = 2;
  inline bool has_id() const;
  inline void clear_id();
  static const int kIdFieldNumber = 2;
  inline ::google::protobuf::uint32 id() const;
  inline void set_id(::google::protobuf::uint32 value);

  // required string parent_name = 3;
  inline bool has_parent_name() const;
  inline void clear_parent_name();
  static const int kParentNameFieldNumber = 3;
  inline const ::std::string& parent_name() const;
  inline void set_parent_name(const ::std::string& value);
  inline void set_parent_name(const char* value);
  inline void set_parent_name(const char* value, size_t size);
  inline ::std::string* mutable_parent_name();
  inline ::std::string* release_parent_name();
  inline void set_allocated_parent_name(::std::string* parent_name);

  // optional uint32 parent_id = 4;
  inline bool has_parent_id() const;
  inline void clear_parent_id();
  static const int kParentIdFieldNumber = 4;
  inline ::google::protobuf::uint32 parent_id() const;
  inline void set_parent_id(::google::protobuf::uint32 value);

  // optional bool cast_shadows = 5;
  inline bool has_cast_shadows() const;
  inline void clear_cast_shadows();
  static const int kCastShadowsFieldNumber = 5;
  inline bool cast_shadows() const;
  inline void set_cast_shadows(bool value);

  // optional double transparency = 6;
  inline bool has_transparency() const;
  inline void clear_transparency();
  static const int kTransparencyFieldNumber = 6;
  inline double transparency() const;
  inline void set_transparency(double value);

  // optional double laser_retro = 7;
  inline bool has_laser_retro() const;
  inline void clear_laser_retro();
  static const int kLaserRetroFieldNumber = 7;
  inline double laser_retro() const;
  inline void set_laser_retro(double value);

  // optional .gazebo.msgs.Pose pose = 8;
  inline bool has_pose() const;
  inline void clear_pose();
  static const int kPoseFieldNumber = 8;
  inline const ::gazebo::msgs::Pose& pose() const;
  inline ::gazebo::msgs::Pose* mutable_pose();
  inline ::gazebo::msgs::Pose* release_pose();
  inline void set_allocated_pose(::gazebo::msgs::Pose* pose);

  // optional .gazebo.msgs.Geometry geometry = 9;
  inline bool has_geometry() const;
  inline void clear_geometry();
  static const int kGeometryFieldNumber = 9;
  inline const ::gazebo::msgs::Geometry& geometry() const;
  inline ::gazebo::msgs::Geometry* mutable_geometry();
  inline ::gazebo::msgs::Geometry* release_geometry();
  inline void set_allocated_geometry(::gazebo::msgs::Geometry* geometry);

  // optional .gazebo.msgs.Material material = 10;
  inline bool has_material() const;
  inline void clear_material();
  static const int kMaterialFieldNumber = 10;
  inline const ::gazebo::msgs::Material& material() const;
  inline ::gazebo::msgs::Material* mutable_material();
  inline ::gazebo::msgs::Material* release_material();
  inline void set_allocated_material(::gazebo::msgs::Material* material);

  // optional bool visible = 11;
  inline bool has_visible() const;
  inline void clear_visible();
  static const int kVisibleFieldNumber = 11;
  inline bool visible() const;
  inline void set_visible(bool value);

  // optional bool delete_me = 12;
  inline bool has_delete_me() const;
  inline void clear_delete_me();
  static const int kDeleteMeFieldNumber = 12;
  inline bool delete_me() const;
  inline void set_delete_me(bool value);

  // optional bool is_static = 13;
  inline bool has_is_static() const;
  inline void clear_is_static();
  static const int kIsStaticFieldNumber = 13;
  inline bool is_static() const;
  inline void set_is_static(bool value);

  // repeated .gazebo.msgs.Plugin plugin = 14;
  inline int plugin_size() const;
  inline void clear_plugin();
  static const int kPluginFieldNumber = 14;
  inline const ::gazebo::msgs::Plugin& plugin(int index) const;
  inline ::gazebo::msgs::Plugin* mutable_plugin(int index);
  inline ::gazebo::msgs::Plugin* add_plugin();
  inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Plugin >&
      plugin() const;
  inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Plugin >*
      mutable_plugin();

  // optional .gazebo.msgs.Vector3d scale = 15;
  inline bool has_scale() const;
  inline void clear_scale();
  static const int kScaleFieldNumber = 15;
  inline const ::gazebo::msgs::Vector3d& scale() const;
  inline ::gazebo::msgs::Vector3d* mutable_scale();
  inline ::gazebo::msgs::Vector3d* release_scale();
  inline void set_allocated_scale(::gazebo::msgs::Vector3d* scale);

  // optional .gazebo.msgs.Visual.Meta meta = 16;
  inline bool has_meta() const;
  inline void clear_meta();
  static const int kMetaFieldNumber = 16;
  inline const ::gazebo::msgs::Visual_Meta& meta() const;
  inline ::gazebo::msgs::Visual_Meta* mutable_meta();
  inline ::gazebo::msgs::Visual_Meta* release_meta();
  inline void set_allocated_meta(::gazebo::msgs::Visual_Meta* meta);

  // optional .gazebo.msgs.Visual.Type type = 17;
  inline bool has_type() const;
  inline void clear_type();
  static const int kTypeFieldNumber = 17;
  inline ::gazebo::msgs::Visual_Type type() const;
  inline void set_type(::gazebo::msgs::Visual_Type value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Visual)
 private:
  inline void set_has_name();
  inline void clear_has_name();
  inline void set_has_id();
  inline void clear_has_id();
  inline void set_has_parent_name();
  inline void clear_has_parent_name();
  inline void set_has_parent_id();
  inline void clear_has_parent_id();
  inline void set_has_cast_shadows();
  inline void clear_has_cast_shadows();
  inline void set_has_transparency();
  inline void clear_has_transparency();
  inline void set_has_laser_retro();
  inline void clear_has_laser_retro();
  inline void set_has_pose();
  inline void clear_has_pose();
  inline void set_has_geometry();
  inline void clear_has_geometry();
  inline void set_has_material();
  inline void clear_has_material();
  inline void set_has_visible();
  inline void clear_has_visible();
  inline void set_has_delete_me();
  inline void clear_has_delete_me();
  inline void set_has_is_static();
  inline void clear_has_is_static();
  inline void set_has_scale();
  inline void clear_has_scale();
  inline void set_has_meta();
  inline void clear_has_meta();
  inline void set_has_type();
  inline void clear_has_type();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::std::string* name_;
  ::std::string* parent_name_;
  ::google::protobuf::uint32 id_;
  ::google::protobuf::uint32 parent_id_;
  double transparency_;
  double laser_retro_;
  ::gazebo::msgs::Pose* pose_;
  ::gazebo::msgs::Geometry* geometry_;
  ::gazebo::msgs::Material* material_;
  bool cast_shadows_;
  bool visible_;
  bool delete_me_;
  bool is_static_;
  int type_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Plugin > plugin_;
  ::gazebo::msgs::Vector3d* scale_;
  ::gazebo::msgs::Visual_Meta* meta_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(17 + 31) / 32];

  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_visual_2eproto();
  friend void protobuf_AssignDesc_visual_2eproto();
  friend void protobuf_ShutdownFile_visual_2eproto();

  void InitAsDefaultInstance();
  static Visual* default_instance_;
};
// ===================================================================


// ===================================================================

// Visual_Meta

// optional int32 layer = 1;
inline bool Visual_Meta::has_layer() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Visual_Meta::set_has_layer() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Visual_Meta::clear_has_layer() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Visual_Meta::clear_layer() {
  layer_ = 0;
  clear_has_layer();
}
inline ::google::protobuf::int32 Visual_Meta::layer() const {
  return layer_;
}
inline void Visual_Meta::set_layer(::google::protobuf::int32 value) {
  set_has_layer();
  layer_ = value;
}

// -------------------------------------------------------------------

// Visual

// required string name = 1;
inline bool Visual::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Visual::set_has_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Visual::clear_has_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Visual::clear_name() {
  if (name_ != &::google::protobuf::internal::kEmptyString) {
    name_->clear();
  }
  clear_has_name();
}
inline const ::std::string& Visual::name() const {
  return *name_;
}
inline void Visual::set_name(const ::std::string& value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Visual::set_name(const char* value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Visual::set_name(const char* value, size_t size) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Visual::mutable_name() {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  return name_;
}
inline ::std::string* Visual::release_name() {
  clear_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = name_;
    name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void Visual::set_allocated_name(::std::string* name) {
  if (name_ != &::google::protobuf::internal::kEmptyString) {
    delete name_;
  }
  if (name) {
    set_has_name();
    name_ = name;
  } else {
    clear_has_name();
    name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  }
}

// optional uint32 id = 2;
inline bool Visual::has_id() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Visual::set_has_id() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Visual::clear_has_id() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Visual::clear_id() {
  id_ = 0u;
  clear_has_id();
}
inline ::google::protobuf::uint32 Visual::id() const {
  return id_;
}
inline void Visual::set_id(::google::protobuf::uint32 value) {
  set_has_id();
  id_ = value;
}

// required string parent_name = 3;
inline bool Visual::has_parent_name() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Visual::set_has_parent_name() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Visual::clear_has_parent_name() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Visual::clear_parent_name() {
  if (parent_name_ != &::google::protobuf::internal::kEmptyString) {
    parent_name_->clear();
  }
  clear_has_parent_name();
}
inline const ::std::string& Visual::parent_name() const {
  return *parent_name_;
}
inline void Visual::set_parent_name(const ::std::string& value) {
  set_has_parent_name();
  if (parent_name_ == &::google::protobuf::internal::kEmptyString) {
    parent_name_ = new ::std::string;
  }
  parent_name_->assign(value);
}
inline void Visual::set_parent_name(const char* value) {
  set_has_parent_name();
  if (parent_name_ == &::google::protobuf::internal::kEmptyString) {
    parent_name_ = new ::std::string;
  }
  parent_name_->assign(value);
}
inline void Visual::set_parent_name(const char* value, size_t size) {
  set_has_parent_name();
  if (parent_name_ == &::google::protobuf::internal::kEmptyString) {
    parent_name_ = new ::std::string;
  }
  parent_name_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Visual::mutable_parent_name() {
  set_has_parent_name();
  if (parent_name_ == &::google::protobuf::internal::kEmptyString) {
    parent_name_ = new ::std::string;
  }
  return parent_name_;
}
inline ::std::string* Visual::release_parent_name() {
  clear_has_parent_name();
  if (parent_name_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = parent_name_;
    parent_name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void Visual::set_allocated_parent_name(::std::string* parent_name) {
  if (parent_name_ != &::google::protobuf::internal::kEmptyString) {
    delete parent_name_;
  }
  if (parent_name) {
    set_has_parent_name();
    parent_name_ = parent_name;
  } else {
    clear_has_parent_name();
    parent_name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  }
}

// optional uint32 parent_id = 4;
inline bool Visual::has_parent_id() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Visual::set_has_parent_id() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Visual::clear_has_parent_id() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Visual::clear_parent_id() {
  parent_id_ = 0u;
  clear_has_parent_id();
}
inline ::google::protobuf::uint32 Visual::parent_id() const {
  return parent_id_;
}
inline void Visual::set_parent_id(::google::protobuf::uint32 value) {
  set_has_parent_id();
  parent_id_ = value;
}

// optional bool cast_shadows = 5;
inline bool Visual::has_cast_shadows() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Visual::set_has_cast_shadows() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Visual::clear_has_cast_shadows() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Visual::clear_cast_shadows() {
  cast_shadows_ = false;
  clear_has_cast_shadows();
}
inline bool Visual::cast_shadows() const {
  return cast_shadows_;
}
inline void Visual::set_cast_shadows(bool value) {
  set_has_cast_shadows();
  cast_shadows_ = value;
}

// optional double transparency = 6;
inline bool Visual::has_transparency() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Visual::set_has_transparency() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Visual::clear_has_transparency() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void Visual::clear_transparency() {
  transparency_ = 0;
  clear_has_transparency();
}
inline double Visual::transparency() const {
  return transparency_;
}
inline void Visual::set_transparency(double value) {
  set_has_transparency();
  transparency_ = value;
}

// optional double laser_retro = 7;
inline bool Visual::has_laser_retro() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void Visual::set_has_laser_retro() {
  _has_bits_[0] |= 0x00000040u;
}
inline void Visual::clear_has_laser_retro() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void Visual::clear_laser_retro() {
  laser_retro_ = 0;
  clear_has_laser_retro();
}
inline double Visual::laser_retro() const {
  return laser_retro_;
}
inline void Visual::set_laser_retro(double value) {
  set_has_laser_retro();
  laser_retro_ = value;
}

// optional .gazebo.msgs.Pose pose = 8;
inline bool Visual::has_pose() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void Visual::set_has_pose() {
  _has_bits_[0] |= 0x00000080u;
}
inline void Visual::clear_has_pose() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void Visual::clear_pose() {
  if (pose_ != NULL) pose_->::gazebo::msgs::Pose::Clear();
  clear_has_pose();
}
inline const ::gazebo::msgs::Pose& Visual::pose() const {
  return pose_ != NULL ? *pose_ : *default_instance_->pose_;
}
inline ::gazebo::msgs::Pose* Visual::mutable_pose() {
  set_has_pose();
  if (pose_ == NULL) pose_ = new ::gazebo::msgs::Pose;
  return pose_;
}
inline ::gazebo::msgs::Pose* Visual::release_pose() {
  clear_has_pose();
  ::gazebo::msgs::Pose* temp = pose_;
  pose_ = NULL;
  return temp;
}
inline void Visual::set_allocated_pose(::gazebo::msgs::Pose* pose) {
  delete pose_;
  pose_ = pose;
  if (pose) {
    set_has_pose();
  } else {
    clear_has_pose();
  }
}

// optional .gazebo.msgs.Geometry geometry = 9;
inline bool Visual::has_geometry() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void Visual::set_has_geometry() {
  _has_bits_[0] |= 0x00000100u;
}
inline void Visual::clear_has_geometry() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void Visual::clear_geometry() {
  if (geometry_ != NULL) geometry_->::gazebo::msgs::Geometry::Clear();
  clear_has_geometry();
}
inline const ::gazebo::msgs::Geometry& Visual::geometry() const {
  return geometry_ != NULL ? *geometry_ : *default_instance_->geometry_;
}
inline ::gazebo::msgs::Geometry* Visual::mutable_geometry() {
  set_has_geometry();
  if (geometry_ == NULL) geometry_ = new ::gazebo::msgs::Geometry;
  return geometry_;
}
inline ::gazebo::msgs::Geometry* Visual::release_geometry() {
  clear_has_geometry();
  ::gazebo::msgs::Geometry* temp = geometry_;
  geometry_ = NULL;
  return temp;
}
inline void Visual::set_allocated_geometry(::gazebo::msgs::Geometry* geometry) {
  delete geometry_;
  geometry_ = geometry;
  if (geometry) {
    set_has_geometry();
  } else {
    clear_has_geometry();
  }
}

// optional .gazebo.msgs.Material material = 10;
inline bool Visual::has_material() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void Visual::set_has_material() {
  _has_bits_[0] |= 0x00000200u;
}
inline void Visual::clear_has_material() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void Visual::clear_material() {
  if (material_ != NULL) material_->::gazebo::msgs::Material::Clear();
  clear_has_material();
}
inline const ::gazebo::msgs::Material& Visual::material() const {
  return material_ != NULL ? *material_ : *default_instance_->material_;
}
inline ::gazebo::msgs::Material* Visual::mutable_material() {
  set_has_material();
  if (material_ == NULL) material_ = new ::gazebo::msgs::Material;
  return material_;
}
inline ::gazebo::msgs::Material* Visual::release_material() {
  clear_has_material();
  ::gazebo::msgs::Material* temp = material_;
  material_ = NULL;
  return temp;
}
inline void Visual::set_allocated_material(::gazebo::msgs::Material* material) {
  delete material_;
  material_ = material;
  if (material) {
    set_has_material();
  } else {
    clear_has_material();
  }
}

// optional bool visible = 11;
inline bool Visual::has_visible() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void Visual::set_has_visible() {
  _has_bits_[0] |= 0x00000400u;
}
inline void Visual::clear_has_visible() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void Visual::clear_visible() {
  visible_ = false;
  clear_has_visible();
}
inline bool Visual::visible() const {
  return visible_;
}
inline void Visual::set_visible(bool value) {
  set_has_visible();
  visible_ = value;
}

// optional bool delete_me = 12;
inline bool Visual::has_delete_me() const {
  return (_has_bits_[0] & 0x00000800u) != 0;
}
inline void Visual::set_has_delete_me() {
  _has_bits_[0] |= 0x00000800u;
}
inline void Visual::clear_has_delete_me() {
  _has_bits_[0] &= ~0x00000800u;
}
inline void Visual::clear_delete_me() {
  delete_me_ = false;
  clear_has_delete_me();
}
inline bool Visual::delete_me() const {
  return delete_me_;
}
inline void Visual::set_delete_me(bool value) {
  set_has_delete_me();
  delete_me_ = value;
}

// optional bool is_static = 13;
inline bool Visual::has_is_static() const {
  return (_has_bits_[0] & 0x00001000u) != 0;
}
inline void Visual::set_has_is_static() {
  _has_bits_[0] |= 0x00001000u;
}
inline void Visual::clear_has_is_static() {
  _has_bits_[0] &= ~0x00001000u;
}
inline void Visual::clear_is_static() {
  is_static_ = false;
  clear_has_is_static();
}
inline bool Visual::is_static() const {
  return is_static_;
}
inline void Visual::set_is_static(bool value) {
  set_has_is_static();
  is_static_ = value;
}

// repeated .gazebo.msgs.Plugin plugin = 14;
inline int Visual::plugin_size() const {
  return plugin_.size();
}
inline void Visual::clear_plugin() {
  plugin_.Clear();
}
inline const ::gazebo::msgs::Plugin& Visual::plugin(int index) const {
  return plugin_.Get(index);
}
inline ::gazebo::msgs::Plugin* Visual::mutable_plugin(int index) {
  return plugin_.Mutable(index);
}
inline ::gazebo::msgs::Plugin* Visual::add_plugin() {
  return plugin_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Plugin >&
Visual::plugin() const {
  return plugin_;
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Plugin >*
Visual::mutable_plugin() {
  return &plugin_;
}

// optional .gazebo.msgs.Vector3d scale = 15;
inline bool Visual::has_scale() const {
  return (_has_bits_[0] & 0x00004000u) != 0;
}
inline void Visual::set_has_scale() {
  _has_bits_[0] |= 0x00004000u;
}
inline void Visual::clear_has_scale() {
  _has_bits_[0] &= ~0x00004000u;
}
inline void Visual::clear_scale() {
  if (scale_ != NULL) scale_->::gazebo::msgs::Vector3d::Clear();
  clear_has_scale();
}
inline const ::gazebo::msgs::Vector3d& Visual::scale() const {
  return scale_ != NULL ? *scale_ : *default_instance_->scale_;
}
inline ::gazebo::msgs::Vector3d* Visual::mutable_scale() {
  set_has_scale();
  if (scale_ == NULL) scale_ = new ::gazebo::msgs::Vector3d;
  return scale_;
}
inline ::gazebo::msgs::Vector3d* Visual::release_scale() {
  clear_has_scale();
  ::gazebo::msgs::Vector3d* temp = scale_;
  scale_ = NULL;
  return temp;
}
inline void Visual::set_allocated_scale(::gazebo::msgs::Vector3d* scale) {
  delete scale_;
  scale_ = scale;
  if (scale) {
    set_has_scale();
  } else {
    clear_has_scale();
  }
}

// optional .gazebo.msgs.Visual.Meta meta = 16;
inline bool Visual::has_meta() const {
  return (_has_bits_[0] & 0x00008000u) != 0;
}
inline void Visual::set_has_meta() {
  _has_bits_[0] |= 0x00008000u;
}
inline void Visual::clear_has_meta() {
  _has_bits_[0] &= ~0x00008000u;
}
inline void Visual::clear_meta() {
  if (meta_ != NULL) meta_->::gazebo::msgs::Visual_Meta::Clear();
  clear_has_meta();
}
inline const ::gazebo::msgs::Visual_Meta& Visual::meta() const {
  return meta_ != NULL ? *meta_ : *default_instance_->meta_;
}
inline ::gazebo::msgs::Visual_Meta* Visual::mutable_meta() {
  set_has_meta();
  if (meta_ == NULL) meta_ = new ::gazebo::msgs::Visual_Meta;
  return meta_;
}
inline ::gazebo::msgs::Visual_Meta* Visual::release_meta() {
  clear_has_meta();
  ::gazebo::msgs::Visual_Meta* temp = meta_;
  meta_ = NULL;
  return temp;
}
inline void Visual::set_allocated_meta(::gazebo::msgs::Visual_Meta* meta) {
  delete meta_;
  meta_ = meta;
  if (meta) {
    set_has_meta();
  } else {
    clear_has_meta();
  }
}

// optional .gazebo.msgs.Visual.Type type = 17;
inline bool Visual::has_type() const {
  return (_has_bits_[0] & 0x00010000u) != 0;
}
inline void Visual::set_has_type() {
  _has_bits_[0] |= 0x00010000u;
}
inline void Visual::clear_has_type() {
  _has_bits_[0] &= ~0x00010000u;
}
inline void Visual::clear_type() {
  type_ = 0;
  clear_has_type();
}
inline ::gazebo::msgs::Visual_Type Visual::type() const {
  return static_cast< ::gazebo::msgs::Visual_Type >(type_);
}
inline void Visual::set_type(::gazebo::msgs::Visual_Type value) {
  assert(::gazebo::msgs::Visual_Type_IsValid(value));
  set_has_type();
  type_ = value;
}


typedef boost::shared_ptr<gazebo::msgs::Visual> VisualPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {

template <>
inline const EnumDescriptor* GetEnumDescriptor< ::gazebo::msgs::Visual_Type>() {
  return ::gazebo::msgs::Visual_Type_descriptor();
}

}  // namespace google
}  // namespace protobuf
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::Visual const> ConstVisualPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_visual_2eproto__INCLUDED
