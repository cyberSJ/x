// A groovy code.
package org.gradle

// We can use this because the root build.gradle script declares the dependency
// on  the gradle api.
import org.gradle.api.DefaultTask
import org.gradle.api.tasks.TaskAction

/**
 * @brief A task that greets.
 *
 * @author skim
 */
class GreetingTask extends DefaultTask {
    String greeting = "default greeting"

    // This specifies that the greet() function will be registered as the
    // function that will be executed when the GreetingTask executes.
    // What happens when there are multiple functions annotated with the
    // @TaskAction?
    @TaskAction
    /**
     * @brief Greets with the configured greeting.
     */
    def greet() {
        println greeting
    }
}
