package com.ssafy.cellcheck.db.repository;

import com.ssafy.cellcheck.db.entity.User;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.Optional;

@Repository
public interface UserRepository extends JpaRepository<User, Long> {
    Optional<User> findByLoginId(String loginId);

    Boolean existsByLoginId(String loginId);
}
