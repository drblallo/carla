import python

cls Walker:
    PyObject underlying

    fun set_relative_target_location(Float x, Float y):
        let args : PyObject[2]
        args[0] = to_pyobject(x)
        args[1] = to_pyobject(y)
        self.underlying.call("set_relative_target_location", args)
    
fun make_walker(PyObject pyobj) -> Walker:
    let walker : Walker
    walker.underlying = pyobj
    return walker

act walker_behaviour(frm Walker walker) -> WalkerBehaviour:
    frm x = 10.0
    while true:
        walker.set_relative_target_location(-4.0, x)
        act reached_location()
        x = -x
