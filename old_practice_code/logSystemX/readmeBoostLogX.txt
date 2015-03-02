
How to integrate Boost.Log into code:
==========================================
Goal: To log in custom format easily.
Need to create a custom wrapper around Boost Log so that we can have
custom format. All these below happens in that custom wrapper.


Create custom logger:
--------------------------
Define severity level using enum.
Inherit a severity & channel logger to use from Boost.
Somehow, %Severity% attribute is not set if you
register_simple_formatter_factory(). 
(or just regular factory).

Let user specify the name of the channel in constructor.
"channel" was already made as an attribute using
BOOST_LOG_ATTRIBUTE_KEYWORD().


Create Boost Test environment:
---------------------------------
Boost.Test can use Boost.Log or any kind of class as a 
setup/cleanup process. The 'setup process' is storing the name
of the test case and test suite so that we can log them.

In order to do this, we create a class (struct, rather) that
can capture the test case and test suite name in the log.
This is the loggedCase struct.


Construct a LoggedCase struct:
-------------------------------------
Get the test suite and test case name (Undocumented in Boost).
test::framework::current_test_case().p_name returns std::string
of the test case name.
Google how to get suite name.
Once these info are stored, its time to register these info as an
boost log attribute keyword (so that we can use the new keywords
in logging format).


Register a totally new attribute:
----------------------------------
we need to add_global_attribute(). This requires the test suite
and test case name, which we got from previous step.
Then we need to use the macro BOOST_LOG_ATTRIBUTE_KEYWORD(), which
uses the keyword that we added as the global attribute.

No.. actually the order is reversed. Once you register an attribute
with the BOOST_LOG_ATTRIBUTE_KEYWORD(), then you can access
the attribute's keyword's get_name() and use it to add_global_attribute(). Use the add_global_attribute() on the log::core.

That macro creates [arbitraryAttribute]_type::value_type, which can be
used as the type in the attributes::constant<> template.
The macro also creates [arbitraryAttribute] which is the 
attribute_keyword class which allows users to use various related
functions such as get_name() to retrieve the name of the attribute.


Customize the Boost.Log sink:
----------------------------------
The sink must have backend type text_file_backend.
(not text_ostream_backend because we need to use
set_file_name_pattern() which only exist in text_file_backend)

Make the sink to save in a file with the [suiteName].log
How to do it is: you obtain a backend and use the backend to
customize the file name in which you want to save the logging file.
Use set_file_name_pattern() on the obtained backend.

I think you need to google some search in order to find
set_file_name_pattern() function.

Make a shared pointer of synchronous_sink.
Obtain a locked_backend() from the synchronous_sink.
Wit the obtained locked_backend, set_file_name_pattern using the
suite name and ".log" extension.
If the code can insert the suite into the std::set of the suite names,
set_open_mode() of backend to out and trunc, otherwise, set the mode
to out and app.

If the .log file already exist, append, otherwise, create new using
set_open_mode().

Customize the Boost.Log formatter:
---------------------------------------
Then it's time to define the formatter. Formatter resides in the sink.
set_formatter with the expressions::format and the desired format.
Register the keywords and use the keywords when creating your desired
format.

Filter also resides in sink. set_filter() with the condition for 
filtering.

In the destructor of the LoggedTestCase, remove_sink() and 
remove_global_attribute() the registered attribute.


Then add filter, and add the customized sink to the core.

