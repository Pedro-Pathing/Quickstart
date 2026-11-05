package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.extension.AfterEachCallback;
import org.junit.jupiter.api.extension.BeforeEachCallback;
import org.junit.jupiter.api.extension.ExtensionContext;

/**
 * Reloads the bundled config before and after every Jupiter test, so a test that changes a config
 * value and forgets to restore it cannot change what the next test sees. Without this, results
 * depended on test order: a tolerance override in one class made turret properties fail only when
 * that class happened to run first. Registered for every test through junit-platform.properties.
 *
 * <p>jqwik property classes restore their own changes in {@code @AfterProperty}.
 */
public class FreshConfigExtension implements BeforeEachCallback, AfterEachCallback {
  @Override
  public void beforeEach(ExtensionContext context) {
    config.reload();
  }

  @Override
  public void afterEach(ExtensionContext context) {
    config.reload();
  }
}
