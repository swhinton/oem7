# Novatel OEM7 C++20

This is a light-weight C++20 library to interface with the Novatel OEM7 GPS receiver.

Why? Novatel EDIE is bloated, difficult to grok, and I don't need most of the features it supports. Also,
conan is *required*... there are other "frameworks" out there for interfacing embedded systems to sensors/actuators,
but most require commitment to a theory of design/architecture. This utilizes the awesome `zpp_bits` to convert bytes
to/from meaningful data structures for consumption in client code. KISS.

I've worked with too many fringe code styles to keep anything consistent so please forgive any insane style.
