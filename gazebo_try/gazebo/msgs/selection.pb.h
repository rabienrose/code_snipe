// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: selection.proto

#ifndef PROTOBUF_selection_2eproto__INCLUDED
#define PROTOBUF_selection_2eproto__INCLUDED

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
#include <google/protobuf/unknown_field_set.h>
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_selection_2eproto();
void protobuf_AssignDesc_selection_2eproto();
void protobuf_ShutdownFile_selection_2eproto();

class Selection;

// ===================================================================

class GZ_MSGS_VISIBLE Selection : public ::google::protobuf::Message {
 public:
  Selection();
  virtual ~Selection();

  Selection(const Selection& from);

  inline Selection& operator=(const Selection& from) {
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
  static const Selection& default_instance();

  void Swap(Selection* other);

  // implements Message ----------------------------------------------

  Selection* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Selection& from);
  void MergeFrom(const Selection& from);
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

  // required uint32 id = 1;
  inline bool has_id() const;
  inline void clear_id();
  static const int kIdFieldNumber = 1;
  inline ::google::protobuf::uint32 id() const;
  inline void set_id(::google::protobuf::uint32 value);

  // required string name = 2;
  inline bool has_name() const;
  inline void clear_name();
  static const int kNameFieldNumber = 2;
  inline const ::std::string& name() const;
  inline void set_name(const ::std::string& value);
  inline void set_name(const char* value);
  inline void set_name(const char* value, size_t size);
  inline ::std::string* mutable_name();
  inline ::std::string* release_name();
  inline void set_allocated_name(::std::string* name);

  // optional bool selected = 3 [default = false];
  inline bool has_selected() const;
  inline void clear_selected();
  static const int kSelectedFieldNumber = 3;
  inline bool selected() const;
  inline void set_selected(bool value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Selection)
 private:
  inline void set_has_id();
  inline void clear_has_id();
  inline void set_has_name();
  inline void clear_has_name();
  inline void set_has_selected();
  inline void clear_has_selected();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::std::string* name_;
  ::google::protobuf::uint32 id_;
  bool selected_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(3 + 31) / 32];

  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_selection_2eproto();
  friend void protobuf_AssignDesc_selection_2eproto();
  friend void protobuf_ShutdownFile_selection_2eproto();

  void InitAsDefaultInstance();
  static Selection* default_instance_;
};
// ===================================================================


// ===================================================================

// Selection

// required uint32 id = 1;
inline bool Selection::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Selection::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Selection::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Selection::clear_id() {
  id_ = 0u;
  clear_has_id();
}
inline ::google::protobuf::uint32 Selection::id() const {
  return id_;
}
inline void Selection::set_id(::google::protobuf::uint32 value) {
  set_has_id();
  id_ = value;
}

// required string name = 2;
inline bool Selection::has_name() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Selection::set_has_name() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Selection::clear_has_name() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Selection::clear_name() {
  if (name_ != &::google::protobuf::internal::kEmptyString) {
    name_->clear();
  }
  clear_has_name();
}
inline const ::std::string& Selection::name() const {
  return *name_;
}
inline void Selection::set_name(const ::std::string& value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Selection::set_name(const char* value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Selection::set_name(const char* value, size_t size) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Selection::mutable_name() {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  return name_;
}
inline ::std::string* Selection::release_name() {
  clear_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = name_;
    name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void Selection::set_allocated_name(::std::string* name) {
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

// optional bool selected = 3 [default = false];
inline bool Selection::has_selected() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Selection::set_has_selected() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Selection::clear_has_selected() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Selection::clear_selected() {
  selected_ = false;
  clear_has_selected();
}
inline bool Selection::selected() const {
  return selected_;
}
inline void Selection::set_selected(bool value) {
  set_has_selected();
  selected_ = value;
}


typedef boost::shared_ptr<gazebo::msgs::Selection> SelectionPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::Selection const> ConstSelectionPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_selection_2eproto__INCLUDED
