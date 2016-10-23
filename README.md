# Granular Media
TP5 for Systems Simulation course from ITBA.

Java Implementation of a granular system
## Build
To build the project, it is necessary to have Maven and Java 1.8 installed.
Then, run

    $ mvn clean package
    
## Execution
To run the program, from the root folder

    $ java -jar core/target/granular-media.jar <arguments>

## Simulation
`help` argument is a highly detailed help menu that show possible usages of the current program.
So, we highly recommend that for using this jar, you may run

    $ java -jar core/target/granular-media.jar help

### Usage examples

Generate static data file

    $ java -jar core/target/granul-media.jar gen static 700 1e-2 1.1e-2 2.5e-3 0.01 10e5 20e5

Generate dynamic data file
    
    $ java -jar core/target/granul-media.jar gen dynamic output/static.dat

Run granular system's simulation

    $ java -jar core/target/granul-media.jar sim output/static.dat output/dynamic.dat .5 1e-7 1e-4 true
    
#### Usage note
To use a recipient instead of a sile, generate a static data file with `diameterOpening = 0`, i.e.,

    $ java -jar core/target/granul-media.jar gen static 700 1e-2 1.1e-2 0 0.01 10e5 20e5
    
System is prepared to stop when it has reached the rest condition **ONLY for the above shown conditions**.

In future versions, you should be able to make it stop for different systems.


