"""Per-cup skill abstraction for the 3-2-1 pyramid build.

A *skill* is one independently executable step of a stacking
sequence.  For the 3-2-1 pyramid that is a single cup: pick it from
the nested source stack and place it at one pyramid slot.  A
controller node owns the ordering and *presents* the start
coordinate (the cup-middle XY); each skill already knows its
destination because that is derivable from the pyramid centre.

Some steps are not robot motions of their own: :class:`ScanSkill`
just runs the existing scan node in a sub-process.  Either way the
package never imports the reference task/runtime/config code.

Existing :mod:`cup_stack.tasks` code is left untouched; this package
is a parallel, finer-grained entry point.
"""
