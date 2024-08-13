#pragma once

#include <istream>

template<typename T>
class peeking_iterator 
{
public:
  explicit peeking_iterator(std::istreambuf_iterator<T>&& stream) :
    _end_count(0),
    _stream(stream),
    _current(*(_stream++)) 
  {}

  peeking_iterator& operator++()
  {
    if (is_end()) return *this;

    if (_end_count == 0) 
    {
      _current = *(_stream++);
    }
    _end_count += (_stream == _end);

    return *this;
}

  inline T operator*() const { return _current; }
  
  inline T peek() const { return (_end_count > 0) ? _current : *_stream; }

  [[nodiscard]] inline bool is_end() const { return (_end_count > 1); }

private:
  static constexpr std::istreambuf_iterator<T> _end {};
  size_t _end_count;
  std::istreambuf_iterator<T> _stream;
  T _current;
};