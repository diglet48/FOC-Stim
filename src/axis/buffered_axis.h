// #ifndef FOCSTIM_BUFFERED_AXIS_H
// #define FOCSTIM_BUFFERED_AXIS_H

// #include <cstdint>
// #include <cstring>
// #include <algorithm>
// #include <CircularBuffer.hpp>    // rlogiacco/CircularBuffer@^1.4.0

// #include "foc_utils.h"

// #define BUFFERED_AXIS_BUFFER_SIZE 32



// class BufferedAxis {
//     /**
//      * Class that allows streaming coordinate data ahead of time.
//      *
//      * Holds a (sorted) buffer of time, value pairs.
//      * If buffer is full, the most recent datapoint is discarded.
//      * Inserting a point causes all points with equal or later timestamp to be lost.
//      * get() linearly interpolates between the nearest 2 points.
//      */
// public:
//     BufferedAxis(float minimum_value, float maximum_value, float default_value) {
//         this->minimum_value = minimum_value;
//         this->maximum_value = maximum_value;
//         this->default_value = default_value;
//     }

//     void add(uint64_t current_timestamp, uint64_t timestamp, float value) {
//         clean_before(current_timestamp);

//         // TODO: jump prevention
//         // TODO: overwrite?

//         // remove any items the are sequenced after the new timestamp
//         // ensures sorting and no repeated timestamps
//         while (data.size()) {
//             if (data.last().timestamp >= timestamp) {
//                 data.pop();
//             } else {
//                 break;
//             }
//         }

//         // if full, discard newest
//         if (data.size() == data.capacity) {
//             data.pop();
//         }

//         data.push({timestamp, value});
//     }

//     void clear() {
//         data.clear();
//     }

//     void clean_before(uint64_t timestamp) {
//         // remove points from the start of the buffer until
//         // at most one point is sequenced before timestamp.
//         while (data.size() >= 2) {
//             if (data[1].timestamp <= timestamp) {
//                 data.shift();
//             } else {
//                 break;
//             }
//         }
//     }

//     float get(uint64_t timestamp) {
//         clean_before(timestamp);
//         float value;

//         if (data.size() == 0) {
//             value = default_value;
//         } else if (data.size() == 1) {
//             value = data.first().value;
//         } else if (data.first().timestamp >= timestamp) {
//             value =  data.first().value;
//         } else {
//             Entry left = data[0];
//             Entry right = data[1];
//             float p = float((timestamp - left.timestamp)) / float((right.timestamp - left.timestamp));
//             value = left.value + p * (right.value - left.value);
//         }

//         return _constrain(value, minimum_value, maximum_value);
//     }


//     float minimum_value;
//     float maximum_value;
//     float default_value;

//     struct Entry {
//         uint64_t timestamp;
//         float value;
//     };

//     CircularBuffer<Entry, BUFFERED_AXIS_BUFFER_SIZE> data;
// };

// #endif