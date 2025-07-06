class ExceptionHandler:
    """
    Print terminal lines when exception is raised.
    Does NOT terminate the program.

    @author: Huanjie Zhang
    """

    @staticmethod
    def raise_illegal_arguments(msg=""):
        print("Illegal argument exception(s) with message: %s", msg)

    @staticmethod
    def raise_illegal_format(msg=""):
        print("Illegal format exception(s) with message: %s", msg)

    @staticmethod
    def raise_index_out_of_range(object_name="", index=0):
        msg = "Encountered index out of range at runtime"
        if object_name != "":
            msg += " with " + object_name + " at index of " + str(index)
        print(msg)
