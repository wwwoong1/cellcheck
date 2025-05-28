package com.ssafy.cellcheck.global.config;

import com.ssafy.cellcheck.global.mqtt.MqttMessageHandler;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.integration.channel.DirectChannel;
import org.springframework.integration.core.MessageProducer;
import org.springframework.integration.mqtt.core.DefaultMqttPahoClientFactory;
import org.springframework.integration.mqtt.core.MqttPahoClientFactory;
import org.springframework.integration.mqtt.inbound.MqttPahoMessageDrivenChannelAdapter;
import org.springframework.integration.mqtt.support.DefaultPahoMessageConverter;
import org.springframework.messaging.MessageChannel;
import org.springframework.messaging.MessageHandler;

import javax.net.ssl.SSLContext;
import javax.net.ssl.TrustManagerFactory;
import java.io.FileInputStream;
import java.security.KeyStore;
import java.security.cert.CertificateFactory;
import java.security.cert.X509Certificate;

@Slf4j
@Configuration
@RequiredArgsConstructor
public class MqttConfig {

    @Value("${mqtt.broker}")
    private String broker;

    @Value("${mqtt.username}")
    private String username;

    @Value("${mqtt.password}")
    private String password;

    @Value("${mqtt.caCertPath}")
    private String caCertPath;

    private static final String CLIENT_ID = "spring-mqtt-client-" + System.currentTimeMillis();

    // 구독할 토픽 목록
    private static final String[] TOPICS = {
            "factory1/machine01/appearance",
            "factory1/machine01/environment",
            "factory1/machine01/discharge",
            "factory1/machine01/system"
    };

    // 수신채널
    @Bean
    public MessageChannel mqttInputChannel() {
        return new DirectChannel();
    }

    // 연결옵션, 인증 ,TLS 설정
    @Bean
    public MqttPahoClientFactory mqttCLientFactory() {
        DefaultMqttPahoClientFactory factory = new DefaultMqttPahoClientFactory();
        MqttConnectOptions options = new MqttConnectOptions();

        // TLS 사용여부 검증
        if (!broker.startsWith("ssl://") && !broker.startsWith("tls://")) {
            log.warn("MQTT 브로커 URL이 ssl:// 또는 tls://로 시작하지 않습니다. TLS 연결을 위해 URL 형식을 확인하세요.");
        }

        // 기본 연결 옵션
        options.setServerURIs(new String[]{broker});
        options.setUserName(username);
        options.setPassword(password.toCharArray());
        options.setCleanSession(true); // 클라이언트 연결 해제 시 세션 정보 삭제
        options.setAutomaticReconnect(true); // 자동 재연결 시도
        options.setConnectionTimeout(30); // 연결 시도 타임아웃

        // SSL/TLS 적용
        try {
            options.setSocketFactory(createSSLSocketFactory());
            log.info("MQTT TLS 설정이 성공적으로 적용되었습니다.");
        } catch (Exception e) {
            log.error("MQTT TLS 설정 중 오류 발생", e);
            throw new RuntimeException("MQTT TLS 설정 실패", e);
        }

        factory.setConnectionOptions(options);
        return factory;


    }

    // SSL 소켓 팩토리 생성
    private javax.net.ssl.SSLSocketFactory createSSLSocketFactory() throws Exception {
        log.info("TLS 인증서 파일 로드 중: {}", caCertPath);

        // CA 인증서 로드
        CertificateFactory cf = CertificateFactory.getInstance("X.509");
        FileInputStream fis = new FileInputStream(caCertPath);
        X509Certificate caCert = (X509Certificate) cf.generateCertificate(fis);
        fis.close();

        // 2. 키스토어 초기화 및 CA 인증서 추가
        KeyStore caKs = KeyStore.getInstance(KeyStore.getDefaultType());
        caKs.load(null, null);  // 빈 키스토어 초기화
        caKs.setCertificateEntry("ca-certificate", caCert);  // CA 인증서 저장

        // 3. 트러스트 매니저 팩토리 초기화 (CA 인증서를 신뢰하도록 설정)
        TrustManagerFactory tmf = TrustManagerFactory.getInstance(TrustManagerFactory.getDefaultAlgorithm());
        tmf.init(caKs);

        // 4. SSL 컨텍스트 생성 및 초기화
        SSLContext sslContext = SSLContext.getInstance("TLSv1.2");
        sslContext.init(null, tmf.getTrustManagers(), null);

        return sslContext.getSocketFactory();


    }

    // 채널 어댑터 구성 ( 브로커 -> 메시지 수신 -> 입력채널 전달)
    @Bean
    public MessageProducer inbound() {
        // 어댑터 생성 (id,factory, topic)
        MqttPahoMessageDrivenChannelAdapter adapter =
                new MqttPahoMessageDrivenChannelAdapter(CLIENT_ID, mqttCLientFactory(), TOPICS);

        // 설정
        adapter.setCompletionTimeout(5000); // 메시지 처리 타임아웃
        adapter.setConverter(new DefaultPahoMessageConverter()); // 메시지 변환기
        adapter.setQos(1); // 최소 1회 전달 보장
        adapter.setOutputChannel(mqttInputChannel()); // 메시지 출력 채널

        return adapter;

    }

    // 수신된 메시지 처리
    @Bean
    @ServiceActivator(inputChannel = "mqttInputChannel")
    public MessageHandler handler() {
        return new MqttMessageHandler();
    }



}
